namespace LPSolverPractice
{
    /// <summary>One "Product Form and Price Out" iteration of the Revised Primal Simplex.</summary>
    public class RevisedIteration
    {
        public int IterationNumber { get; set; }
        public List<string> BasisNames { get; set; } = new();
        public double[][] Binv { get; set; } = Array.Empty<double[]>();   // current basis inverse (product form)
        public double[] BBar { get; set; } = Array.Empty<double>();        // Binv * b  (current RHS / basic values)
        public double[] CB { get; set; } = Array.Empty<double>();          // objective coefficients of the basic columns
        public double[] Y { get; set; } = Array.Empty<double>();           // simplex multipliers, y^T = cB^T * Binv
        public double[] ReducedCosts { get; set; } = Array.Empty<double>(); // "price out" row: cj - y.Aj for every column
        public string EnteringName { get; set; } = "";
        public double[]? ProductFormColumn { get; set; }                  // Binv * A_entering (the "product form" column)
        public string LeavingName { get; set; } = "";
        public string Note { get; set; } = "";
    }

    public class RevisedSimplexResult
    {
        public StandardForm Form { get; set; } = new();
        public List<RevisedIteration> Iterations { get; set; } = new();
        public bool Infeasible { get; set; }
        public bool Unbounded { get; set; }
        public int[] FinalBasis { get; set; } = Array.Empty<int>();
        public double[][] FinalBinv { get; set; } = Array.Empty<double[]>();
        public Dictionary<string, double> OriginalSolution { get; set; } = new();
        public double ObjectiveValue { get; set; }
        public List<double> ShadowPrices { get; set; } = new();
    }

    /// <summary>
    /// Revised Primal Simplex Algorithm, Big-M variant.
    ///
    /// Unlike SimplexSolver.cs (which row-reduces one big tableau every pivot), this keeps
    /// only the current basis inverse B^-1 ("product form of the inverse") and derives
    /// everything else -- reduced costs, the entering column, the current solution -- from
    /// it on demand. That derivation step is exactly the "Price Out" the brief asks for:
    ///   y^T   = cB^T . B^-1                (simplex multipliers -- these ARE the shadow prices)
    ///   cj-zj = cj - y^T . Aj              (reduced cost / price-out row, all columns)
    ///   Aj~   = B^-1 . Aj                  (product-form column of the entering variable)
    ///   bBar  = B^-1 . b                   (current basic solution)
    /// After a pivot, B^-1 is updated with a single elementary row operation (an "eta"
    /// matrix), rather than recomputed from scratch -- that update IS the product-form
    /// step. The end result (objective value, x*, feasibility) is mathematically identical
    /// to SimplexSolver's tableau simplex; only the bookkeeping shown at each iteration
    /// differs, which is the point of having both algorithms in the menu.
    /// </summary>
    public static class RevisedSimplex
    {
        private const double BigM = 1_000_000.0;
        private const double Eps = 1e-9;
        private const int MaxIterations = 500;

        public static RevisedSimplexResult Solve(LPModel model)
        {
            var form = StandardFormBuilder.Build(model);
            int nCons = form.A.Length;
            int nTotal = form.C.Length;

            // internal-maximise objective, Big-M penalty on artificial columns
            var cInternal = form.C.Select(v => v * form.DirSign).ToArray();
            foreach (var a in form.ArtificialColumns) cInternal[a] = -BigM;

            // B^-1 starts as the identity: the initial basis columns (slacks / artificials)
            // are, by construction, the columns of the identity matrix.
            var binv = Identity(nCons);
            var basis = (int[])form.Basis.Clone();

            var result = new RevisedSimplexResult { Form = form };
            var iterations = new List<RevisedIteration>();

            for (int iter = 0; iter < MaxIterations; iter++)
            {
                var cB = basis.Select(b => cInternal[b]).ToArray();
                var y = VecTimesMat(cB, binv);           // y^T = cB^T . B^-1
                var bBar = MatTimesVec(binv, form.B);     // current basic values

                var reduced = new double[nTotal];
                for (int j = 0; j < nTotal; j++)
                {
                    double yAj = 0.0;
                    for (int r = 0; r < nCons; r++) yAj += y[r] * form.A[r][j];
                    reduced[j] = cInternal[j] - yAj;
                }

                int entering = -1;
                double best = Eps;
                var basisSet = new HashSet<int>(basis);
                for (int j = 0; j < nTotal; j++)
                {
                    if (basisSet.Contains(j)) continue;
                    if (reduced[j] > best) { best = reduced[j]; entering = j; }
                }

                var snap = new RevisedIteration
                {
                    IterationNumber = iterations.Count,
                    BasisNames = basis.Select(b => form.Names[b]).ToList(),
                    Binv = binv.Select(row => (double[])row.Clone()).ToArray(),
                    BBar = (double[])bBar.Clone(),
                    CB = (double[])cB.Clone(),
                    Y = (double[])y.Clone(),
                    ReducedCosts = (double[])reduced.Clone(),
                    Note = entering == -1 ? "Optimal (no positive reduced cost remains)" : "Price-out"
                };

                if (entering == -1) { iterations.Add(snap); break; }

                var dCol = MatTimesVec(binv, form.A.Select(row => row[entering]).ToArray()); // product-form column

                int leaving = -1;
                double bestRatio = double.PositiveInfinity;
                for (int r = 0; r < nCons; r++)
                {
                    if (dCol[r] > Eps)
                    {
                        double ratio = bBar[r] / dCol[r];
                        if (ratio < bestRatio - Eps) { bestRatio = ratio; leaving = r; }
                    }
                }

                snap.EnteringName = form.Names[entering];
                snap.ProductFormColumn = (double[])dCol.Clone();

                if (leaving == -1)
                {
                    snap.Note = $"Unbounded: column '{form.Names[entering]}' has no positive entry to ratio-test against.";
                    iterations.Add(snap);
                    result.Unbounded = true;
                    break;
                }

                snap.LeavingName = form.Names[basis[leaving]];
                snap.Note = $"Entering '{form.Names[entering]}', leaving '{form.Names[basis[leaving]]}' (row {leaving + 1})";
                iterations.Add(snap);

                // Eta update: B^-1_new = E . B^-1, where E is the identity except column 'leaving'
                double piv = dCol[leaving];
                var eta = Identity(nCons);
                for (int r = 0; r < nCons; r++)
                    eta[r][leaving] = (r == leaving) ? 1.0 / piv : -dCol[r] / piv;
                binv = MatTimesMat(eta, binv);

                basis[leaving] = entering;
            }

            result.Iterations = iterations;
            result.FinalBasis = basis;
            result.FinalBinv = binv;

            if (result.Unbounded) return result;

            var finalBBar = MatTimesVec(binv, form.B);
            for (int r = 0; r < nCons; r++)
            {
                if (form.ArtificialColumns.Contains(basis[r]) && finalBBar[r] > 1e-6)
                    result.Infeasible = true;
            }
            if (result.Infeasible) return result;

            var workingValues = new double[nTotal];
            for (int r = 0; r < nCons; r++) workingValues[basis[r]] = finalBBar[r];

            var originalValues = new double[model.VariableCount];
            for (int c = 0; c < form.WorkingColumnCount; c++)
                originalValues[form.Columns[c].OriginalVariableIndex] += form.Columns[c].Multiplier * workingValues[c];

            for (int i = 0; i < model.VariableCount; i++)
                result.OriginalSolution[$"x{i + 1}"] = Math.Round(originalValues[i], 3);

            double objValue = 0.0;
            for (int i = 0; i < model.VariableCount; i++)
                objValue += model.ObjectiveCoefficients[i] * originalValues[i];
            result.ObjectiveValue = Math.Round(objValue, 3);

            // Shadow prices = y (the simplex multipliers of the final basis), one per ORIGINAL
            // constraint, converted back to the original max/min sense via DirSign.
            var cBFinal = basis.Select(b => cInternal[b]).ToArray();
            var yFinal = VecTimesMat(cBFinal, binv);
            for (int i = 0; i < form.ConstraintCount; i++)
                result.ShadowPrices.Add(Math.Round(form.DirSign * yFinal[i], 3));

            return result;
        }

        /// <summary>Text report: canonical form, every Product-Form / Price-Out iteration, result and shadow prices.</summary>
        public static string BuildReport(LPModel model, RevisedSimplexResult result)
        {
            var sb = new System.Text.StringBuilder();
            var names = result.Form.Names;

            sb.AppendLine("=== ORIGINAL MODEL ===");
            using (var sw = new StringWriter(sb)) model.Print(sw);
            sb.AppendLine();

            sb.AppendLine("=== CANONICAL FORM ===");
            sb.AppendLine((model.IsMax ? "max  z = " : "min  z = ") +
                string.Join(" ", names.Select((nm, jc) => $"{LPModel.FormatSigned(jc < result.Form.C.Length ? result.Form.C[jc] : 0.0)}{nm}")));
            sb.AppendLine("s.t.");
            for (int r = 0; r < result.Form.A.Length; r++)
            {
                var terms = new List<string>();
                for (int c = 0; c < names.Count; c++)
                {
                    double cellVal = c < result.Form.A[r].Length ? result.Form.A[r][c] : 0.0;
                    if (Math.Abs(cellVal) > 1e-9) terms.Add($"{LPModel.FormatSigned(cellVal)}{names[c]}");
                }
                sb.AppendLine("     " + string.Join(" ", terms) + $" = {Math.Round(result.Form.B[r], 3)}");
            }
            sb.AppendLine("     " + string.Join(", ", names) + " >= 0");
            sb.AppendLine();

            sb.AppendLine("=== REVISED PRIMAL SIMPLEX -- PRODUCT FORM & PRICE-OUT ITERATIONS ===");
            foreach (var it in result.Iterations)
            {
                sb.AppendLine($"--- Iteration {it.IterationNumber}: {it.Note} ---");
                sb.AppendLine("Basis: " + string.Join(", ", it.BasisNames));
                sb.AppendLine("B^-1 (product form of the basis inverse):");
                foreach (var row in it.Binv)
                    sb.AppendLine("     " + string.Join("  ", row.Select(v => Math.Round(v, 3).ToString("0.000").PadLeft(9))));
                sb.AppendLine("cB (basic costs): " + string.Join(", ", it.CB.Select(v => Math.Round(v, 3))));
                sb.AppendLine("y = cB . B^-1 (simplex multipliers / shadow prices this iteration): " +
                    string.Join(", ", it.Y.Select(v => Math.Round(v, 3))));
                sb.AppendLine("bBar = B^-1 . b (current basic values): " + string.Join(", ", it.BBar.Select(v => Math.Round(v, 3))));
                sb.AppendLine("Price-out row (cj - y.Aj) for every column:");
                for (int j = 0; j < names.Count; j++)
                    sb.Append($"  {names[j]}={Math.Round(it.ReducedCosts[j], 3)}");
                sb.AppendLine();
                if (it.ProductFormColumn != null)
                {
                    sb.AppendLine($"Entering '{it.EnteringName}', product-form column B^-1.A_entering = " +
                        string.Join(", ", it.ProductFormColumn.Select(v => Math.Round(v, 3))));
                    if (!string.IsNullOrEmpty(it.LeavingName))
                        sb.AppendLine($"Leaving '{it.LeavingName}'");
                }
                sb.AppendLine();
            }

            sb.AppendLine("=== RESULT ===");
            if (result.Infeasible)
            {
                sb.AppendLine("INFEASIBLE: an artificial variable remains positive in the optimal basis.");
            }
            else if (result.Unbounded)
            {
                sb.AppendLine("UNBOUNDED: the entering column had no positive entry to limit it (ratio test failed).");
            }
            else
            {
                sb.AppendLine($"Optimal ({(model.IsMax ? "max" : "min")}) objective value z = {result.ObjectiveValue.ToString("0.000")}");
                foreach (var kv in result.OriginalSolution)
                    sb.AppendLine($"  {kv.Key} = {kv.Value.ToString("0.000")}");
                sb.AppendLine();
                sb.AppendLine("Shadow prices (= final y vector, one per original constraint):");
                for (int i = 0; i < result.ShadowPrices.Count; i++)
                    sb.AppendLine($"  constraint {i + 1}: {result.ShadowPrices[i].ToString("0.000")}");
            }

            return sb.ToString();
        }

        // ---------- small dense linear-algebra helpers (no external dependency needed) ----------
        private static double[][] Identity(int n)
        {
            var m = new double[n][];
            for (int i = 0; i < n; i++)
            {
                m[i] = new double[n];
                m[i][i] = 1.0;
            }
            return m;
        }

        private static double[] MatTimesVec(double[][] m, double[] v)
        {
            int n = m.Length;
            var result = new double[n];
            for (int r = 0; r < n; r++)
            {
                double sum = 0.0;
                for (int c = 0; c < v.Length; c++) sum += m[r][c] * v[c];
                result[r] = sum;
            }
            return result;
        }

        private static double[] VecTimesMat(double[] v, double[][] m)
        {
            int n = m.Length; // v has length n (rows of m)
            int cols = n == 0 ? 0 : m[0].Length;
            var result = new double[cols];
            for (int c = 0; c < cols; c++)
            {
                double sum = 0.0;
                for (int r = 0; r < n; r++) sum += v[r] * m[r][c];
                result[c] = sum;
            }
            return result;
        }

        private static double[][] MatTimesMat(double[][] a, double[][] b)
        {
            int n = a.Length;
            int m = b.Length == 0 ? 0 : b[0].Length;
            int k = b.Length;
            var result = new double[n][];
            for (int r = 0; r < n; r++)
            {
                result[r] = new double[m];
                for (int c = 0; c < m; c++)
                {
                    double sum = 0.0;
                    for (int t = 0; t < k; t++) sum += a[r][t] * b[t][c];
                    result[r][c] = sum;
                }
            }
            return result;
        }
    }
}
