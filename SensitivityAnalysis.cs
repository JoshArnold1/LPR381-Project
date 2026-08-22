using System.Text;

namespace LPSolverPractice
{
    /// <summary>
    /// Sensitivity analysis operations, all driven off the FINAL tableau that SimplexSolver
    /// (Big-M Primal Simplex) produces for the model's current optimal solution.
    ///
    /// Design choice, spelled out up front: every "display the range of ..." operation below
    /// uses the standard closed-form ranging formulas (reduced costs, the current basis's
    /// row entries, and the B^-1 column that sits under each constraint's own slack/
    /// artificial column in the final tableau -- these formulas were checked against
    /// SimplexSolver's own shadow-price values and against direct perturb-and-resolve
    /// experiments before being written here, so trust them to be internally consistent
    /// with SimplexSolver.cs's Big-M convention). Every "apply and display a change"
    /// operation, on the other hand, just edits a CLONE of the model and calls
    /// SimplexSolver.Solve on it again from scratch -- that is slower than patching the
    /// existing tableau in place, but it can never be wrong, because it reuses the exact
    /// same simplex code the rest of the menu already relies on.
    ///
    /// SCOPE NOTE: these operations assume the queried variable has sign restriction '+',
    /// 'int' or 'bin' (i.e. it maps to exactly one working column with multiplier +1).
    /// A '-' or 'urs' variable is expanded into one or two DIFFERENT working columns by
    /// SimplexSolver, and ranging those correctly needs extra bookkeeping this practice
    /// build doesn't do -- see FindWorkingColumn below, which reports that limitation
    /// instead of silently giving a wrong answer.
    /// </summary>
    public static class SensitivityAnalysis
    {
        private const double Eps = 1e-9;

        private static int FindWorkingColumn(LPModel model, SimplexResult result, int originalVarIndex, out string? error)
        {
            error = null;
            if (originalVarIndex < 0 || originalVarIndex >= model.VariableCount)
            {
                error = "Invalid variable index.";
                return -1;
            }
            var restriction = model.SignRestrictions.Count > originalVarIndex ? model.SignRestrictions[originalVarIndex] : "+";
            if (restriction is "urs" or "-")
            {
                error = $"x{originalVarIndex + 1} is restricted '{restriction}', which SimplexSolver expands into more " +
                         "than one working column. This build's ranging/column operations only support '+', 'int' and " +
                         "'bin' variables -- see the SCOPE NOTE at the top of SensitivityAnalysis.cs.";
                return -1;
            }
            int idx = result.Columns.FindIndex(wc => wc.OriginalVariableIndex == originalVarIndex);
            if (idx == -1) error = $"Could not find a working column for x{originalVarIndex + 1}.";
            return idx;
        }

        private static string FormatBound(double v)
        {
            if (double.IsPositiveInfinity(v)) return "+infinity";
            if (double.IsNegativeInfinity(v)) return "-infinity";
            return Math.Round(v, 3).ToString("0.###");
        }

        private static LPModel CloneModel(LPModel model)
        {
            var clone = new LPModel { IsMax = model.IsMax };
            clone.ObjectiveCoefficients = new List<double>(model.ObjectiveCoefficients);
            clone.SignRestrictions = new List<string>(model.SignRestrictions);
            foreach (var c in model.Constraints)
                clone.Constraints.Add(new Constraint
                {
                    Coefficients = new List<double>(c.Coefficients),
                    Relation = c.Relation,
                    Rhs = c.Rhs
                });
            return clone;
        }

        // ================= 1-2: NON-BASIC VARIABLE (cost) =================

        public static string RangeNonBasicVariable(LPModel model, SimplexResult result, int originalVarIndex)
        {
            var sb = new StringBuilder();
            int col = FindWorkingColumn(model, result, originalVarIndex, out var err);
            if (col == -1) { sb.AppendLine(err); return sb.ToString(); }
            if (result.BasisColumnIndex.Contains(col))
            {
                sb.AppendLine($"x{originalVarIndex + 1} is currently BASIC -- use the basic-variable range option instead.");
                return sb.ToString();
            }

            double rc = result.Iterations[^1].ObjectiveRow[col]; // internal reduced cost, >= 0 at optimum
            double c0 = model.ObjectiveCoefficients[originalVarIndex];

            sb.AppendLine($"x{originalVarIndex + 1} is non-basic (current value 0). Reduced cost = {Math.Round(rc, 3)}.");
            if (model.IsMax)
                sb.AppendLine($"c{originalVarIndex + 1} = {c0:0.###} can range over (-infinity, {FormatBound(c0 + rc)}] " +
                               "before it becomes profitable to bring x into the basis.");
            else
                sb.AppendLine($"c{originalVarIndex + 1} = {c0:0.###} can range over [{FormatBound(c0 - rc)}, +infinity) " +
                               "before it becomes profitable to bring x into the basis.");
            return sb.ToString();
        }

        public static string ApplyNonBasicVariableChange(LPModel model, int originalVarIndex, double newCoefficient)
        {
            var clone = CloneModel(model);
            clone.ObjectiveCoefficients[originalVarIndex] = newCoefficient;
            var newResult = SimplexSolver.Solve(clone);

            var sb = new StringBuilder();
            sb.AppendLine($"Changed c{originalVarIndex + 1} from {model.ObjectiveCoefficients[originalVarIndex]:0.###} to {newCoefficient:0.###} " +
                           "and re-solved the model from scratch:");
            sb.AppendLine(OutputWriter.BuildReport(clone, newResult));
            return sb.ToString();
        }

        // ================= 3-4: BASIC VARIABLE (cost) =================

        public static string RangeBasicVariable(LPModel model, SimplexResult result, int originalVarIndex)
        {
            var sb = new StringBuilder();
            int col = FindWorkingColumn(model, result, originalVarIndex, out var err);
            if (col == -1) { sb.AppendLine(err); return sb.ToString(); }

            int row = result.BasisColumnIndex.IndexOf(col);
            if (row == -1)
            {
                sb.AppendLine($"x{originalVarIndex + 1} is currently NON-basic -- use the non-basic range option instead.");
                return sb.ToString();
            }

            var body = result.Iterations[^1].Body;
            var objRow = result.Iterations[^1].ObjectiveRow;
            int nTotal = objRow.Length - 1;
            double dirSign = model.IsMax ? 1.0 : -1.0;
            var basisSet = new HashSet<int>(result.BasisColumnIndex);

            double deltaIntMax = double.PositiveInfinity, deltaIntMin = double.NegativeInfinity;
            for (int j = 0; j < nTotal; j++)
            {
                if (basisSet.Contains(j)) continue;
                double a = body[row][j];
                if (a > Eps) deltaIntMax = Math.Min(deltaIntMax, objRow[j] / a);
                else if (a < -Eps) deltaIntMin = Math.Max(deltaIntMin, objRow[j] / a);
            }

            double c0 = model.ObjectiveCoefficients[originalVarIndex];
            double lower, upper;
            if (dirSign > 0) { lower = c0 + deltaIntMin; upper = c0 + deltaIntMax; }
            else { lower = c0 - deltaIntMax; upper = c0 - deltaIntMin; }

            double curVal = result.OriginalSolution.TryGetValue($"x{originalVarIndex + 1}", out var v) ? v : 0.0;
            sb.AppendLine($"x{originalVarIndex + 1} is basic with value {curVal:0.###} (row {row + 1} of the final tableau).");
            sb.AppendLine($"c{originalVarIndex + 1} = {c0:0.###} can range over [{FormatBound(lower)}, {FormatBound(upper)}] " +
                           "while the current basis stays optimal.");
            return sb.ToString();
        }

        public static string ApplyBasicVariableChange(LPModel model, int originalVarIndex, double newCoefficient)
            => ApplyNonBasicVariableChange(model, originalVarIndex, newCoefficient); // identical mechanics: edit + re-solve

        // ================= 5-6: CONSTRAINT RHS =================

        public static string RangeRhs(LPModel model, SimplexResult result, int constraintIndex)
        {
            var sb = new StringBuilder();
            if (constraintIndex < 0 || constraintIndex >= model.Constraints.Count)
            {
                sb.AppendLine("Invalid constraint index.");
                return sb.ToString();
            }

            var form = StandardFormBuilder.Build(model); // only used to find the row's initial identity column
            int identityCol = form.Basis[constraintIndex];
            var body = result.Iterations[^1].Body;
            int nCons = body.Length;

            var lows = new List<double>();
            var highs = new List<double>();
            for (int r = 0; r < nCons; r++)
            {
                double colVal = body[r][identityCol];
                double rhsVal = body[r][^1];
                if (colVal > Eps) lows.Add(-rhsVal / colVal);
                else if (colVal < -Eps) highs.Add(-rhsVal / colVal);
            }
            double lowDelta = lows.Count > 0 ? lows.Max() : double.NegativeInfinity;
            double highDelta = highs.Count > 0 ? highs.Min() : double.PositiveInfinity;

            double b0 = model.Constraints[constraintIndex].Rhs;
            sb.AppendLine($"RHS of constraint {constraintIndex + 1} (currently {b0:0.###}) can range over " +
                           $"[{FormatBound(b0 + lowDelta)}, {FormatBound(b0 + highDelta)}] while the current basis stays optimal.");
            sb.AppendLine($"Shadow price over this range: {result.ShadowPrices[constraintIndex]:0.###} " +
                           "(objective changes by shadow price * delta for any delta inside the range).");
            return sb.ToString();
        }

        public static string ApplyRhsChange(LPModel model, int constraintIndex, double newRhs)
        {
            var clone = CloneModel(model);
            clone.Constraints[constraintIndex].Rhs = newRhs;
            var newResult = SimplexSolver.Solve(clone);

            var sb = new StringBuilder();
            sb.AppendLine($"Changed RHS of constraint {constraintIndex + 1} from {model.Constraints[constraintIndex].Rhs:0.###} to {newRhs:0.###} " +
                           "and re-solved the model from scratch:");
            sb.AppendLine(OutputWriter.BuildReport(clone, newResult));
            return sb.ToString();
        }

        // ================= 7-8: A NON-BASIC COLUMN'S TECHNOLOGICAL COEFFICIENT =================

        public static string RangeNonBasicColumnCoefficient(LPModel model, SimplexResult result, int originalVarIndex, int constraintIndex)
        {
            var sb = new StringBuilder();
            int col = FindWorkingColumn(model, result, originalVarIndex, out var err);
            if (col == -1) { sb.AppendLine(err); return sb.ToString(); }
            if (result.BasisColumnIndex.Contains(col))
            {
                sb.AppendLine($"x{originalVarIndex + 1} is currently BASIC -- this option is for non-basic columns only.");
                return sb.ToString();
            }
            if (constraintIndex < 0 || constraintIndex >= model.Constraints.Count)
            {
                sb.AppendLine("Invalid constraint index.");
                return sb.ToString();
            }

            double dirSign = model.IsMax ? 1.0 : -1.0;
            double yInternal = result.ShadowPrices[constraintIndex] * dirSign; // back out the internal (max-convention) dual value
            double rc = result.Iterations[^1].ObjectiveRow[col];
            double a0 = model.Constraints[constraintIndex].Coefficients[originalVarIndex];

            sb.AppendLine($"a[{constraintIndex + 1}][x{originalVarIndex + 1}] is currently {a0:0.###}.");
            if (Math.Abs(yInternal) < Eps)
            {
                sb.AppendLine("This constraint's shadow price is 0, so this coefficient can change freely " +
                               "(within this test) without affecting optimality of the current basis.");
            }
            else if (yInternal > 0)
            {
                double deltaUpper = rc / yInternal;
                sb.AppendLine($"It can range over (-infinity, {FormatBound(a0 + deltaUpper)}] before x{originalVarIndex + 1} " +
                               "becomes attractive to enter the basis.");
            }
            else
            {
                double deltaLower = rc / yInternal;
                sb.AppendLine($"It can range over [{FormatBound(a0 + deltaLower)}, +infinity) before x{originalVarIndex + 1} " +
                               "becomes attractive to enter the basis.");
            }
            return sb.ToString();
        }

        public static string ApplyColumnCoefficientChange(LPModel model, int originalVarIndex, int constraintIndex, double newCoefficient)
        {
            var clone = CloneModel(model);
            clone.Constraints[constraintIndex].Coefficients[originalVarIndex] = newCoefficient;
            var newResult = SimplexSolver.Solve(clone);

            var sb = new StringBuilder();
            sb.AppendLine($"Changed a[{constraintIndex + 1}][x{originalVarIndex + 1}] from " +
                           $"{model.Constraints[constraintIndex].Coefficients[originalVarIndex]:0.###} to {newCoefficient:0.###} " +
                           "and re-solved the model from scratch:");
            sb.AppendLine(OutputWriter.BuildReport(clone, newResult));
            return sb.ToString();
        }

        // ================= 9: ADD A NEW ACTIVITY (decision variable) =================

        public static string AddNewActivity(LPModel model, SimplexResult result, double newObjCoefficient,
            List<double> newConstraintCoefficients, string signRestriction)
        {
            var sb = new StringBuilder();
            if (newConstraintCoefficients.Count != model.Constraints.Count)
            {
                sb.AppendLine($"Expected {model.Constraints.Count} constraint coefficient(s) for the new activity, " +
                               $"got {newConstraintCoefficients.Count}.");
                return sb.ToString();
            }

            double dirSign = model.IsMax ? 1.0 : -1.0;
            double yDotA = 0.0;
            for (int k = 0; k < model.Constraints.Count; k++)
                yDotA += (result.ShadowPrices[k] * dirSign) * newConstraintCoefficients[k];
            double rcInternal = newObjCoefficient * dirSign - yDotA;

            sb.AppendLine($"Reduced cost of the proposed new activity against the CURRENT optimal basis: {Math.Round(rcInternal, 3)} " +
                           "(internal, max-convention: positive means attractive).");
            if (rcInternal <= Eps)
                sb.AppendLine("This is <= 0, so the new activity would NOT improve the current optimal solution -- it stays optimal at value 0 for the new activity.");
            else
                sb.AppendLine("This is > 0, so the new activity WOULD improve the solution -- re-solving the augmented model:");

            var clone = CloneModel(model);
            clone.ObjectiveCoefficients.Add(newObjCoefficient);
            for (int k = 0; k < clone.Constraints.Count; k++)
                clone.Constraints[k].Coefficients.Add(newConstraintCoefficients[k]);
            clone.SignRestrictions.Add(signRestriction);
            var newResult = SimplexSolver.Solve(clone);

            sb.AppendLine();
            sb.AppendLine("=== Augmented model (existing model + the new activity) re-solved from scratch ===");
            sb.AppendLine(OutputWriter.BuildReport(clone, newResult));
            return sb.ToString();
        }

        // ================= 10: ADD A NEW CONSTRAINT =================

        public static string AddNewConstraint(LPModel model, SimplexResult result, List<double> coefficients, string relation, double rhs)
        {
            var sb = new StringBuilder();
            if (coefficients.Count != model.VariableCount)
            {
                sb.AppendLine($"Expected {model.VariableCount} coefficient(s) for the new constraint, got {coefficients.Count}.");
                return sb.ToString();
            }

            double lhs = 0.0;
            for (int i = 0; i < model.VariableCount; i++)
            {
                double v = result.OriginalSolution.TryGetValue($"x{i + 1}", out var vv) ? vv : 0.0;
                lhs += coefficients[i] * v;
            }
            bool satisfied = relation switch
            {
                "<=" => lhs <= rhs + 1e-6,
                ">=" => lhs >= rhs - 1e-6,
                "=" => Math.Abs(lhs - rhs) < 1e-6,
                _ => false
            };

            sb.AppendLine($"Current optimal solution gives LHS = {Math.Round(lhs, 3)} for the new constraint ({relation} {rhs:0.###}).");
            if (satisfied)
            {
                sb.AppendLine("The current optimal solution already satisfies this constraint -- it stays optimal, nothing changes.");
                return sb.ToString();
            }

            sb.AppendLine("The current optimal solution VIOLATES this constraint -- re-solving with it added:");
            var clone = CloneModel(model);
            clone.Constraints.Add(new Constraint { Coefficients = new List<double>(coefficients), Relation = relation, Rhs = rhs });
            var newResult = SimplexSolver.Solve(clone);

            sb.AppendLine();
            sb.AppendLine(OutputWriter.BuildReport(clone, newResult));
            return sb.ToString();
        }

        // ================= 11: SHADOW PRICES =================

        public static string DisplayShadowPrices(LPModel model, SimplexResult result)
        {
            var sb = new StringBuilder();
            sb.AppendLine("Shadow prices (dual values), one per original constraint, in the order given in the input file:");
            for (int i = 0; i < result.ShadowPrices.Count && i < model.Constraints.Count; i++)
                sb.AppendLine($"  constraint {i + 1}  [{model.Constraints[i]}]  ->  {result.ShadowPrices[i]:0.###}");
            return sb.ToString();
        }
    }
}
