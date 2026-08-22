using System.Text;

namespace LPSolverPractice
{
    /// <summary>
    /// Formats the canonical form, every tableau iteration, the final solution and the
    /// shadow prices as text -- used both for console display and for the output file
    /// ("export all results to an output text file", all values rounded to 3 decimals).
    /// </summary>
    public static class OutputWriter
    {
        public static string BuildReport(LPModel model, SimplexResult result)
        {
            var allColumnNames = result.AllColumnNames;
            var sb = new StringBuilder();

            sb.AppendLine("=== ORIGINAL MODEL ===");
            using (var sw = new StringWriter(sb)) model.Print(sw);
            sb.AppendLine();

            sb.AppendLine("=== CANONICAL FORM (standard form used by the simplex algorithm) ===");
            sb.AppendLine(BuildCanonicalFormText(model, result, allColumnNames));
            sb.AppendLine();

            sb.AppendLine("=== TABLEAU ITERATIONS (Primal Simplex, Big-M method) ===");
            foreach (var snap in result.Iterations)
                sb.AppendLine(FormatSnapshot(snap, allColumnNames));

            sb.AppendLine("=== RESULT ===");
            if (result.Infeasible)
            {
                sb.AppendLine("INFEASIBLE: an artificial variable remains positive in the optimal basis.");
                sb.AppendLine("There is no point that satisfies every constraint simultaneously.");
            }
            else if (result.Unbounded)
            {
                sb.AppendLine("UNBOUNDED: the entering column had no positive entry to limit it (ratio test failed).");
                sb.AppendLine("The objective can be improved without limit.");
            }
            else
            {
                sb.AppendLine($"Optimal ({(model.IsMax ? "max" : "min")}) objective value z = {result.ObjectiveValue.ToString("0.000")}");
                foreach (var kv in result.OriginalSolution)
                    sb.AppendLine($"  {kv.Key} = {kv.Value.ToString("0.000")}");

                sb.AppendLine();
                sb.AppendLine("Shadow prices (one per original constraint, in the order given in the input file):");
                for (int i = 0; i < result.ShadowPrices.Count; i++)
                    sb.AppendLine($"  constraint {i + 1}: {result.ShadowPrices[i].ToString("0.000")}");

                bool hasIntOrBin = model.SignRestrictions.Any(r => r is "int" or "bin");
                if (hasIntOrBin)
                {
                    sb.AppendLine();
                    sb.AppendLine("NOTE: this model has 'int'/'bin' variables. Primal Simplex only solves the");
                    sb.AppendLine("continuous relaxation above -- it does not enforce integrality. Use the");
                    sb.AppendLine("Branch & Bound / Knapsack algorithm for a true integer-feasible solution.");
                }
            }

            return sb.ToString();
        }

        private static string BuildCanonicalFormText(LPModel model, SimplexResult result, List<string> allColumnNames)
        {
            var sb = new StringBuilder();
            var workingCols = result.Columns;
            int nWorking = workingCols.Count;

            var objTerms = new List<string>();
            for (int c = 0; c < nWorking; c++)
            {
                double coeff = model.ObjectiveCoefficients[workingCols[c].OriginalVariableIndex] * workingCols[c].Multiplier;
                objTerms.Add($"{LPModel.FormatSigned(coeff)}{workingCols[c].Name}");
            }
            for (int c = nWorking; c < allColumnNames.Count; c++)
                objTerms.Add($"+0{allColumnNames[c]}");

            sb.AppendLine((model.IsMax ? "max  z = " : "min  z = ") + string.Join(" ", objTerms));
            sb.AppendLine("s.t.");

            foreach (var snap in result.Iterations.Take(1)) // initial tableau IS the canonical form's constraint rows
            {
                for (int r = 0; r < snap.Body.Length; r++)
                {
                    var terms = new List<string>();
                    for (int c = 0; c < allColumnNames.Count; c++)
                    {
                        double v = snap.Body[r][c];
                        if (Math.Abs(v) > 1e-9)
                            terms.Add($"{LPModel.FormatSigned(v)}{allColumnNames[c]}");
                    }
                    double rhs = snap.Body[r][^1];
                    sb.AppendLine("     " + string.Join(" ", terms) + $" = {Math.Round(rhs, 3)}");
                }
            }

            sb.AppendLine("     " + string.Join(", ", allColumnNames) + " >= 0");
            return sb.ToString();
        }

        private static string FormatSnapshot(TableauSnapshot snap, List<string> allColumnNames)
        {
            var sb = new StringBuilder();
            sb.AppendLine($"--- Iteration {snap.IterationNumber}: {snap.Note} ---");

            var header = new List<string> { "Basis" };
            header.AddRange(allColumnNames);
            header.Add("RHS");

            var rowsText = new List<List<string>> { header };

            for (int r = 0; r < snap.Body.Length; r++)
            {
                var row = new List<string> { snap.BasisNames[r] };
                foreach (var v in snap.Body[r]) row.Add(Math.Round(v, 3).ToString("0.000"));
                rowsText.Add(row);
            }

            var objRow = new List<string> { "z" };
            foreach (var v in snap.ObjectiveRow) objRow.Add(Math.Round(v, 3).ToString("0.000"));
            rowsText.Add(objRow);

            int cols = header.Count;
            var widths = new int[cols];
            for (int c = 0; c < cols; c++)
                widths[c] = rowsText.Max(row => row[c].Length) + 2;

            foreach (var row in rowsText)
            {
                for (int c = 0; c < cols; c++)
                    sb.Append(row[c].PadLeft(widths[c]));
                sb.AppendLine();
            }

            return sb.ToString();
        }
    }
}
