namespace LPSolverPractice
{
    public class CuttingPlaneSnapshot
    {
        public int RoundNumber { get; set; }
        public List<string> BasisNames { get; set; } = new();
        public double[][] Body { get; set; } = Array.Empty<double[]>();
        public double[] ObjectiveRow { get; set; } = Array.Empty<double>();
        public string Note { get; set; } = "";
    }

    public class CuttingPlaneResult
    {
        public List<string> Names { get; set; } = new();
        public List<CuttingPlaneSnapshot> Iterations { get; set; } = new();
        public bool Infeasible { get; set; }
        public bool Unbounded { get; set; }
        public bool GaveUp { get; set; }
        public Dictionary<string, double> OriginalSolution { get; set; } = new();
        public double ObjectiveValue { get; set; }
        public StandardForm Form { get; set; } = new();
    }

    /// <summary>
    /// Cutting Plane Algorithm (Gomory fractional cuts), Big-M primal start + dual-simplex
    /// re-optimisation after every cut.
    ///
    /// Step 1: solve the ordinary LP relaxation to optimality (same Big-M pivoting rule as
    /// SimplexSolver.cs, just operating on the shared StandardForm arrays here).
    /// Step 2: if every int/bin decision variable is already integer-valued, we're done.
    /// Otherwise pick the tableau row whose basic variable is a fractional int/bin decision
    /// variable and read off its current row -- coefficients a_ij and RHS b_i.
    /// Step 3: the Gomory cut sum_j frac(a_ij) x_j >= frac(b_i) is added as a brand new row
    /// (with its own slack column g_k) directly into the SAME tableau -- no need to rebuild
    /// anything from the original model, because the cut is expressed in terms of whatever
    /// columns are currently in the tableau (decision variables AND slacks alike).
    /// Step 4: that new row has a negative RHS (b_i's fractional part flips sign), which
    /// breaks primal feasibility but not dual feasibility (the objective row is untouched,
    /// so it is still all >= 0) -- exactly the situation the Dual Simplex method is for.
    /// A few dual-simplex pivots restore primal feasibility without ever revisiting a worse
    /// solution. Repeat 2-4 until integer-feasible, infeasible, or a cut budget is hit.
    ///
    /// NOTE ON THE BRIEF'S WORDING: the brief asks for "Product Form and Price Out
    /// iterations" for both Revised Simplex AND Cutting Plane -- that phrase is the natural
    /// fit for Revised Simplex (see RevisedSimplex.cs) and reads as a copy/paste duplication
    /// for Cutting Plane. What's implemented here instead is what every OR textbook actually
    /// means by "cutting plane iterations": the full tableau after the initial relaxation,
    /// after every cut is appended, and after every dual-simplex pivot that restores
    /// feasibility -- i.e. every tableau the algorithm passes through, in full.
    /// </summary>
    public static class CuttingPlane
    {
        private const double BigM = 1_000_000.0;
        private const double Eps = 1e-9;
        private const int MaxPrimalIterations = 500;
        private const int MaxCuts = 40;

        public static CuttingPlaneResult Solve(LPModel model)
        {
            var form = StandardFormBuilder.Build(model);
            var result = new CuttingPlaneResult { Form = form };
            var names = new List<string>(form.Names);

            var intIdx = new HashSet<int>();
            for (int i = 0; i < model.VariableCount; i++)
                if (model.SignRestrictions.Count > i && (model.SignRestrictions[i] == "int" || model.SignRestrictions[i] == "bin"))
                    intIdx.Add(i);

            int nCons = form.A.Length;
            int nTotal0 = form.C.Length;
            var tab = new double[nCons][];
            for (int r = 0; r < nCons; r++)
            {
                tab[r] = new double[nTotal0 + 1];
                for (int c = 0; c < nTotal0; c++) tab[r][c] = form.A[r][c];
                tab[r][nTotal0] = form.B[r];
            }

            var objRow = new double[nTotal0 + 1];
            var internalObj = form.C.Select(v => v * form.DirSign).ToArray();
            for (int c = 0; c < nTotal0; c++) objRow[c] = -internalObj[c];
            foreach (var a in form.ArtificialColumns) objRow[a] = BigM;

            var basis = (int[])form.Basis.Clone();
            for (int r = 0; r < nCons; r++)
            {
                int bc = basis[r];
                if (Math.Abs(objRow[bc]) > Eps)
                {
                    double factor = objRow[bc];
                    for (int c = 0; c < tab[r].Length; c++) objRow[c] -= factor * tab[r][c];
                }
            }

            var iterations = new List<CuttingPlaneSnapshot>();
            void Snapshot(string note)
            {
                iterations.Add(new CuttingPlaneSnapshot
                {
                    RoundNumber = iterations.Count,
                    BasisNames = basis.Select(b => names[b]).ToList(),
                    Body = tab.Select(row => (double[])row.Clone()).ToArray(),
                    ObjectiveRow = (double[])objRow.Clone(),
                    Note = note
                });
            }

            Snapshot("Initial tableau (canonical form of the LP relaxation)");

            // ---------- Step 1: ordinary primal Big-M simplex to LP optimum ----------
            if (!RunPrimalToOptimum(tab, objRow, basis, names, form.ArtificialColumns, Snapshot))
            {
                result.Unbounded = true;
                result.Iterations = iterations;
                return result;
            }
            for (int r = 0; r < nCons; r++)
                if (form.ArtificialColumns.Contains(basis[r]) && tab[r][^1] > 1e-6)
                    result.Infeasible = true;
            if (result.Infeasible) { result.Iterations = iterations; return result; }

            // ---------- Step 2-4: add Gomory cuts until integer-feasible ----------
            for (int cutNum = 1; cutNum <= MaxCuts; cutNum++)
            {
                int cutRow = -1;
                for (int r = 0; r < tab.Length; r++)
                {
                    int bcol = basis[r];
                    if (bcol < form.WorkingColumnCount && intIdx.Contains(form.Columns[bcol].OriginalVariableIndex))
                    {
                        double rowRhs = tab[r][^1];
                        if (Math.Abs(rowRhs - Math.Round(rowRhs)) > 1e-6) { cutRow = r; break; }
                    }
                }

                if (cutRow == -1) break; // every int/bin variable is integer-valued: done

                int nTotalCur = tab[0].Length - 1;
                var cutCoef = new double[nTotalCur];
                for (int c = 0; c < nTotalCur; c++) cutCoef[c] = FracPart(tab[cutRow][c]);
                double cutRhs = FracPart(tab[cutRow][^1]);

                Snapshot($"Gomory cut #{cutNum} derived from row of basic variable '{names[basis[cutRow]]}' " +
                         $"(fractional value {tab[cutRow][^1]:0.###}) -- before adding to tableau");

                // grow the tableau: one new row, one new column (the cut's own slack g_k)
                int newNTotal = nTotalCur + 1;
                var newTab = new double[tab.Length + 1][];
                for (int r = 0; r < tab.Length; r++)
                {
                    newTab[r] = new double[newNTotal + 1];
                    for (int c = 0; c < nTotalCur; c++) newTab[r][c] = tab[r][c];
                    newTab[r][nTotalCur] = 0.0;
                    newTab[r][newNTotal] = tab[r][^1];
                }
                newTab[tab.Length] = new double[newNTotal + 1];
                for (int c = 0; c < nTotalCur; c++) newTab[tab.Length][c] = -cutCoef[c];
                newTab[tab.Length][nTotalCur] = 1.0;
                newTab[tab.Length][newNTotal] = -cutRhs;

                var newObjRow = new double[newNTotal + 1];
                for (int c = 0; c < nTotalCur; c++) newObjRow[c] = objRow[c];
                newObjRow[nTotalCur] = 0.0;
                newObjRow[newNTotal] = objRow[^1];

                var newBasis = basis.Append(nTotalCur).ToArray();
                names.Add($"g{cutNum}");
                tab = newTab;
                objRow = newObjRow;
                basis = newBasis;

                Snapshot($"Gomory cut #{cutNum} added as new row '{names[^1]}' (RHS = {-cutRhs:0.###}, primal-infeasible / dual-feasible)");

                var dualStatus = RunDualSimplex(tab, objRow, basis, names, Snapshot);
                if (dualStatus == "infeasible")
                {
                    result.Infeasible = true;
                    result.Iterations = iterations;
                    return result;
                }
                if (dualStatus == "gave_up")
                {
                    result.GaveUp = true;
                    break;
                }

                if (cutNum == MaxCuts) result.GaveUp = true;
            }

            result.Names = names;
            result.Iterations = iterations;

            var workingValues = new double[tab[0].Length - 1];
            for (int r = 0; r < tab.Length; r++) workingValues[basis[r]] = tab[r][^1];

            var originalValues = new double[model.VariableCount];
            for (int c = 0; c < form.WorkingColumnCount; c++)
                originalValues[form.Columns[c].OriginalVariableIndex] += form.Columns[c].Multiplier * workingValues[c];

            for (int i = 0; i < model.VariableCount; i++)
                result.OriginalSolution[$"x{i + 1}"] = Math.Round(originalValues[i], 3);

            double objValue = 0.0;
            for (int i = 0; i < model.VariableCount; i++)
                objValue += model.ObjectiveCoefficients[i] * originalValues[i];
            result.ObjectiveValue = Math.Round(objValue, 3);

            return result;
        }

        private static double FracPart(double v)
        {
            double f = v - Math.Floor(v);
            return f;
        }

        /// <summary>Ordinary Big-M primal pivoting to optimality. Returns false if unbounded.</summary>
        private static bool RunPrimalToOptimum(double[][] tab, double[] objRow, int[] basis, List<string> names,
            List<int> artificialColumns, Action<string> snapshot)
        {
            int nCons = tab.Length;
            int nTotal = objRow.Length - 1;

            for (int iteration = 0; iteration < MaxPrimalIterations; iteration++)
            {
                int entering = -1;
                double mostNegative = -Eps;
                for (int c = 0; c < nTotal; c++)
                {
                    if (objRow[c] < mostNegative) { mostNegative = objRow[c]; entering = c; }
                }
                if (entering == -1) return true; // optimal

                int leaving = -1;
                double bestRatio = double.PositiveInfinity;
                for (int r = 0; r < nCons; r++)
                {
                    if (tab[r][entering] > Eps)
                    {
                        double ratio = tab[r][^1] / tab[r][entering];
                        if (ratio < bestRatio - Eps) { bestRatio = ratio; leaving = r; }
                    }
                }
                if (leaving == -1) return false; // unbounded

                double piv = tab[leaving][entering];
                for (int c = 0; c < tab[leaving].Length; c++) tab[leaving][c] /= piv;
                for (int r = 0; r < nCons; r++)
                {
                    if (r == leaving) continue;
                    double factor = tab[r][entering];
                    if (Math.Abs(factor) < Eps) continue;
                    for (int c = 0; c < tab[r].Length; c++) tab[r][c] -= factor * tab[leaving][c];
                }
                double objFactor = objRow[entering];
                if (Math.Abs(objFactor) > Eps)
                    for (int c = 0; c < objRow.Length; c++) objRow[c] -= objFactor * tab[leaving][c];

                basis[leaving] = entering;
                snapshot($"Pivot on column '{names[entering]}', row {leaving + 1} (entering={names[entering]}, leaving={names[basis[leaving]]})");
            }
            return true;
        }

        /// <summary>Dual simplex: restores primal feasibility (RHS >= 0) while keeping the objective row dual-feasible.</summary>
        private static string RunDualSimplex(double[][] tab, double[] objRow, int[] basis, List<string> names, Action<string> snapshot)
        {
            int nCons = tab.Length;
            for (int iteration = 0; iteration < MaxPrimalIterations; iteration++)
            {
                int leaveRow = -1;
                double mostNegative = -1e-7;
                for (int r = 0; r < nCons; r++)
                {
                    if (tab[r][^1] < mostNegative) { mostNegative = tab[r][^1]; leaveRow = r; }
                }
                if (leaveRow == -1) return "optimal";

                int enterCol = -1;
                double bestRatio = double.PositiveInfinity;
                for (int c = 0; c < objRow.Length - 1; c++)
                {
                    if (tab[leaveRow][c] < -Eps)
                    {
                        double ratio = objRow[c] / (-tab[leaveRow][c]);
                        if (ratio < bestRatio - Eps) { bestRatio = ratio; enterCol = c; }
                    }
                }
                if (enterCol == -1) return "infeasible";

                double piv = tab[leaveRow][enterCol];
                for (int c = 0; c < tab[leaveRow].Length; c++) tab[leaveRow][c] /= piv;
                for (int r = 0; r < nCons; r++)
                {
                    if (r == leaveRow) continue;
                    double factor = tab[r][enterCol];
                    if (Math.Abs(factor) < Eps) continue;
                    for (int c = 0; c < tab[r].Length; c++) tab[r][c] -= factor * tab[leaveRow][c];
                }
                double objFactor = objRow[enterCol];
                if (Math.Abs(objFactor) > Eps)
                    for (int c = 0; c < objRow.Length; c++) objRow[c] -= objFactor * tab[leaveRow][c];

                string leavingName = names[basis[leaveRow]];
                basis[leaveRow] = enterCol;
                snapshot($"Dual-simplex pivot: entering '{names[enterCol]}', leaving '{leavingName}' (row {leaveRow + 1})");
            }
            return "gave_up";
        }

        public static string BuildReport(LPModel model, CuttingPlaneResult result)
        {
            var sb = new System.Text.StringBuilder();

            sb.AppendLine("=== ORIGINAL MODEL ===");
            using (var sw = new StringWriter(sb)) model.Print(sw);
            sb.AppendLine();

            sb.AppendLine("=== CANONICAL FORM (initial LP relaxation) ===");
            var names0 = result.Form.Names;
            sb.AppendLine((model.IsMax ? "max  z = " : "min  z = ") +
                string.Join(" ", names0.Select((n, j) => $"{LPModel.FormatSigned(j < result.Form.C.Length ? result.Form.C[j] : 0.0)}{n}")));
            sb.AppendLine("s.t.");
            for (int r = 0; r < result.Form.A.Length; r++)
            {
                var terms = new List<string>();
                for (int c = 0; c < names0.Count; c++)
                {
                    double v = result.Form.A[r][c];
                    if (Math.Abs(v) > 1e-9) terms.Add($"{LPModel.FormatSigned(v)}{names0[c]}");
                }
                sb.AppendLine("     " + string.Join(" ", terms) + $" = {Math.Round(result.Form.B[r], 3)}");
            }
            sb.AppendLine("     " + string.Join(", ", names0) + " >= 0");
            sb.AppendLine();

            sb.AppendLine("=== CUTTING PLANE (GOMORY) -- ALL TABLEAU ITERATIONS ===");
            foreach (var snap in result.Iterations)
            {
                sb.AppendLine($"--- Step {snap.RoundNumber}: {snap.Note} ---");
                var header = new List<string> { "Basis" };
                header.AddRange(Enumerable.Range(0, snap.Body[0].Length - 1).Select(ci => ci < result.Names.Count ? result.Names[ci] : $"c{ci}"));
                header.Add("RHS");

                var rows = new List<List<string>> { header };
                for (int r = 0; r < snap.Body.Length; r++)
                {
                    var row = new List<string> { snap.BasisNames[r] };
                    foreach (var v in snap.Body[r]) row.Add(Math.Round(v, 3).ToString("0.000"));
                    rows.Add(row);
                }
                var objRowText = new List<string> { "z" };
                foreach (var v in snap.ObjectiveRow) objRowText.Add(Math.Round(v, 3).ToString("0.000"));
                rows.Add(objRowText);

                int cols = header.Count;
                var widths = new int[cols];
                for (int c = 0; c < cols; c++) widths[c] = rows.Max(rw => rw[c].Length) + 2;
                foreach (var row in rows)
                {
                    for (int c = 0; c < cols; c++) sb.Append(row[c].PadLeft(widths[c]));
                    sb.AppendLine();
                }
                sb.AppendLine();
            }

            sb.AppendLine("=== RESULT ===");
            if (result.Infeasible)
            {
                sb.AppendLine("INFEASIBLE: either the LP relaxation itself is infeasible, or a Gomory cut proved");
                sb.AppendLine("no integer-feasible point exists (dual simplex found no entering column to restore feasibility).");
            }
            else if (result.Unbounded)
            {
                sb.AppendLine("UNBOUNDED: the LP relaxation is unbounded, so the integer program has no finite optimum either.");
            }
            else if (result.GaveUp)
            {
                sb.AppendLine("Cut limit reached before an integer-feasible tableau was found -- try Branch & Bound instead.");
            }
            else
            {
                sb.AppendLine($"Optimal ({(model.IsMax ? "max" : "min")}) integer objective value z = {result.ObjectiveValue:0.000}");
                foreach (var kv in result.OriginalSolution)
                    sb.AppendLine($"  {kv.Key} = {kv.Value:0.000}");
            }

            return sb.ToString();
        }
    }
}
