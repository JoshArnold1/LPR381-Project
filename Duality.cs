namespace LPSolverPractice
{
    public class DualBuildResult
    {
        public LPModel Dual { get; set; } = new();
        public List<string> Notes { get; set; } = new();
    }

    /// <summary>
    /// Builds and solves the dual of an LPModel, using the general (not just symmetric
    /// max/&lt;=/non-negative) duality table:
    ///
    ///   primal max            | primal min
    ///   ---------------------+---------------------
    ///   constraint &lt;=  -> y&gt;=0 | constraint &gt;=  -> y&gt;=0
    ///   constraint &gt;=  -> y&lt;=0 | constraint &lt;=  -> y&lt;=0
    ///   constraint  =  -> y urs | constraint  =  -> y urs
    ///   var x&gt;=0 -> dual &gt;=   | var x&gt;=0 -> dual &lt;=
    ///   var x&lt;=0 -> dual &lt;=   | var x&lt;=0 -> dual &gt;=
    ///   var x urs -> dual  =   | var x urs -> dual  =
    ///
    /// dual objective coefficients = primal RHS values (one dual variable per primal
    /// constraint); dual constraint RHS values = primal objective coefficients (one dual
    /// constraint per primal variable); the constraint matrix is transposed.
    ///
    /// 'int'/'bin' primal variables are treated as ordinary '+' for this purpose: classical
    /// LP duality is a continuous-LP theorem, so what's really being dualised is the LP
    /// relaxation of the model (this is called out explicitly in the report).
    /// </summary>
    public static class Duality
    {
        public static DualBuildResult BuildDual(LPModel primal)
        {
            var notes = new List<string>();
            int m = primal.Constraints.Count;
            int n = primal.VariableCount;

            var dual = new LPModel { IsMax = !primal.IsMax };
            dual.ObjectiveCoefficients = primal.Constraints.Select(c => c.Rhs).ToList();

            for (int j = 0; j < n; j++)
            {
                var row = new List<double>();
                for (int i = 0; i < m; i++) row.Add(primal.Constraints[i].Coefficients[j]);

                var restriction = primal.SignRestrictions.Count > j ? primal.SignRestrictions[j] : "+";
                if (restriction is "int" or "bin")
                    notes.Add($"x{j + 1} is '{restriction}' -- duality is a continuous-LP theorem, so it is treated as an " +
                              "ordinary non-negative ('+') variable here (the dual of the LP relaxation).");

                bool nonneg = restriction is "+" or "int" or "bin";
                bool nonpos = restriction == "-";

                string relation = primal.IsMax
                    ? (nonneg ? ">=" : nonpos ? "<=" : "=")
                    : (nonneg ? "<=" : nonpos ? ">=" : "=");

                dual.Constraints.Add(new Constraint { Coefficients = row, Relation = relation, Rhs = primal.ObjectiveCoefficients[j] });
            }

            for (int i = 0; i < m; i++)
            {
                var rel = primal.Constraints[i].Relation;
                string sign = primal.IsMax
                    ? (rel == "<=" ? "+" : rel == ">=" ? "-" : "urs")
                    : (rel == ">=" ? "+" : rel == "<=" ? "-" : "urs");
                dual.SignRestrictions.Add(sign);
            }

            return new DualBuildResult { Dual = dual, Notes = notes };
        }

        /// <summary>Builds the dual, solves both models, and checks strong duality (equal optimal objective values).</summary>
        public static string BuildReport(LPModel primal, SimplexResult primalResult)
        {
            var sb = new System.Text.StringBuilder();
            var built = BuildDual(primal);
            var dual = built.Dual;

            sb.AppendLine("=== PRIMAL MODEL ===");
            using (var sw = new StringWriter(sb)) primal.Print(sw);
            sb.AppendLine();

            sb.AppendLine("=== DUAL MODEL ===");
            using (var sw = new StringWriter(sb)) dual.Print(sw);
            foreach (var note in built.Notes) sb.AppendLine("NOTE: " + note);
            sb.AppendLine();

            sb.AppendLine("=== SOLVING THE DUAL (Primal Simplex, Big-M) ===");
            var dualResult = SimplexSolver.Solve(dual);
            sb.AppendLine(OutputWriter.BuildReport(dual, dualResult));
            sb.AppendLine();

            sb.AppendLine("=== DUALITY CHECK ===");
            if (primalResult.Infeasible || primalResult.Unbounded || dualResult.Infeasible || dualResult.Unbounded)
            {
                sb.AppendLine("At least one of primal/dual is infeasible or unbounded, so strong duality (equal finite optimal");
                sb.AppendLine("objectives) cannot apply. LP duality theory says: if the primal is unbounded, the dual is");
                sb.AppendLine("infeasible (and, generically, vice versa).");
                sb.AppendLine($"Primal: {(primalResult.Infeasible ? "INFEASIBLE" : primalResult.Unbounded ? "UNBOUNDED" : "solved")}. " +
                               $"Dual: {(dualResult.Infeasible ? "INFEASIBLE" : dualResult.Unbounded ? "UNBOUNDED" : "solved")}.");
            }
            else
            {
                double zP = primalResult.ObjectiveValue;
                double zD = dualResult.ObjectiveValue;
                sb.AppendLine($"Primal optimal z = {zP:0.000}");
                sb.AppendLine($"Dual optimal   w = {zD:0.000}");
                if (Math.Abs(zP - zD) < 1e-3)
                {
                    sb.AppendLine("STRONG DUALITY holds: primal and dual optimal objective values are equal, exactly as LP");
                    sb.AppendLine("duality theory guarantees whenever both the primal and dual are feasible and bounded.");
                }
                else
                {
                    sb.AppendLine("Objective values differ. For a correctly-paired primal/dual LP this should not happen -- a");
                    sb.AppendLine("gap here points to a bug (e.g. in the model or the dual construction), not to ordinary 'weak'");
                    sb.AppendLine("duality, which only ever describes intermediate FEASIBLE (not necessarily optimal) points.");
                }
            }

            return sb.ToString();
        }
    }
}
