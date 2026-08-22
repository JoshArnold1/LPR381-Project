namespace LPSolverPractice
{
    /// <summary>One working (decision) column and how it maps back to an original variable.</summary>
    public class WorkingColumn
    {
        public string Name { get; set; } = "";
        public int OriginalVariableIndex { get; set; } // index into model.ObjectiveCoefficients
        public double Multiplier { get; set; } = 1.0;  // original value contribution = Multiplier * workingValue
        public bool IsSlackSurplusOrArtificial { get; set; }
        public bool IsArtificial { get; set; }
    }

    /// <summary>A full copy of the tableau + objective row at one point in the algorithm, for display.</summary>
    public class TableauSnapshot
    {
        public int IterationNumber { get; set; }
        public List<string> BasisNames { get; set; } = new();
        public double[][] Body { get; set; } = Array.Empty<double[]>(); // [row][col], last col = RHS
        public double[] ObjectiveRow { get; set; } = Array.Empty<double>(); // last entry = RHS of obj row
        public string Note { get; set; } = "";
    }

    public class SimplexResult
    {
        public List<WorkingColumn> Columns { get; set; } = new();
        public List<TableauSnapshot> Iterations { get; set; } = new();
        public bool Infeasible { get; set; }
        public bool Unbounded { get; set; }
        public Dictionary<string, double> OriginalSolution { get; set; } = new();
        public double ObjectiveValue { get; set; }
        public List<double> ShadowPrices { get; set; } = new(); // one per original constraint, sign per max-convention
        public List<int> BasisColumnIndex { get; set; } = new(); // final basis, one per row
        public List<string> AllColumnNames { get; set; } = new(); // working vars + slack/surplus/artificial, in tableau column order
    }

    /// <summary>
    /// Big-M Primal Simplex Algorithm.
    ///
    /// This is written to be read, not to be fast: every iteration is copied into
    /// a TableauSnapshot so the console / output file can show "all tableau iterations"
    /// as required by the brief. It handles &lt;=, &gt;= and = constraints in one pass
    /// via the classic Big-M method, and expands '-', 'urs' and 'bin' sign restrictions
    /// before building the standard (canonical) form.
    ///
    /// NOTE (scope): this solves the *continuous* relaxation. 'int' and 'bin' variables
    /// are only bounded (bin gets an explicit xi &lt;= 1 row) -- integrality itself is the
    /// job of the Branch &amp; Bound / Cutting Plane algorithms, which are not implemented
    /// here. See README.md for what's included vs. left for you to build.
    /// </summary>
    public static class SimplexSolver
    {
        private const double BigM = 1_000_000.0;
        private const double Eps = 1e-9;
        private const int MaxIterations = 500;

        public static SimplexResult Solve(LPModel model)
        {
            // ---------- Step 1: expand sign-restricted variables into working columns ----------
            var columns = new List<WorkingColumn>();
            for (int i = 0; i < model.VariableCount; i++)
            {
                var restriction = model.SignRestrictions.Count > i ? model.SignRestrictions[i] : "+";
                switch (restriction)
                {
                    case "-":
                        // x_i <= 0  =>  substitute x_i = -y_i , y_i >= 0
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}", OriginalVariableIndex = i, Multiplier = -1.0 });
                        break;
                    case "urs":
                        // free variable => x_i = x_i+ - x_i- , both >= 0
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}+", OriginalVariableIndex = i, Multiplier = 1.0 });
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}-", OriginalVariableIndex = i, Multiplier = -1.0 });
                        break;
                    default: // "+", "int", "bin" all start life as an ordinary non-negative column
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}", OriginalVariableIndex = i, Multiplier = 1.0 });
                        break;
                }
            }

            // ---------- Step 2: build the working objective + constraint rows over these columns ----------
            int nWorking = columns.Count;
            var workingObj = new double[nWorking];
            for (int c = 0; c < nWorking; c++)
                workingObj[c] = model.ObjectiveCoefficients[columns[c].OriginalVariableIndex] * columns[c].Multiplier;

            var rows = new List<(double[] coeffs, string relation, double rhs)>();
            foreach (var con in model.Constraints)
            {
                var row = new double[nWorking];
                for (int c = 0; c < nWorking; c++)
                    row[c] = con.Coefficients[columns[c].OriginalVariableIndex] * columns[c].Multiplier;
                rows.Add((row, con.Relation, con.Rhs));
            }

            // 'bin' variables need an explicit upper bound row: x_i <= 1
            for (int i = 0; i < model.VariableCount; i++)
            {
                if (model.SignRestrictions.Count > i && model.SignRestrictions[i] == "bin")
                {
                    var row = new double[nWorking];
                    int colIdx = columns.FindIndex(c => c.OriginalVariableIndex == i);
                    row[colIdx] = 1.0;
                    rows.Add((row, "<=", 1.0));
                }
            }

            // Normalise: flip any row with a negative RHS so RHS >= 0 (keeps Big-M bookkeeping simple)
            for (int r = 0; r < rows.Count; r++)
            {
                if (rows[r].rhs < 0)
                {
                    var (coeffs, relation, rhs) = rows[r];
                    var flipped = coeffs.Select(v => -v).ToArray();
                    var newRelation = relation == "<=" ? ">=" : relation == ">=" ? "<=" : "=";
                    rows[r] = (flipped, newRelation, -rhs);
                }
            }

            int nCons = rows.Count;

            // ---------- Step 3: add slack / surplus / artificial columns (this IS the canonical form) ----------
            var allNames = columns.Select(c => c.Name).ToList();
            int slackCount = 0, surplusCount = 0, artificialCount = 0;
            var extraColumnKind = new List<string>(); // "slack" | "surplus" | "artificial", parallel to columns added after nWorking

            foreach (var (_, relation, _) in rows)
            {
                if (relation == "<=") { slackCount++; }
                else if (relation == ">=") { surplusCount++; artificialCount++; }
                else { artificialCount++; }
            }

            int nTotal = nWorking + slackCount + surplusCount + artificialCount;
            var tableau = new double[nCons][];
            for (int r = 0; r < nCons; r++) tableau[r] = new double[nTotal + 1]; // +1 for RHS

            var basis = new int[nCons];
            var artificialColumns = new List<int>();
            int nextCol = nWorking;
            int sCounter = 1, eCounter = 1, aCounter = 1;

            for (int r = 0; r < nCons; r++)
            {
                var (coeffs, relation, rhs) = rows[r];
                for (int c = 0; c < nWorking; c++) tableau[r][c] = coeffs[c];
                tableau[r][nTotal] = rhs;

                if (relation == "<=")
                {
                    allNames.Add($"s{sCounter}");
                    tableau[r][nextCol] = 1.0;
                    basis[r] = nextCol;
                    nextCol++; sCounter++;
                }
                else if (relation == ">=")
                {
                    allNames.Add($"e{eCounter}");
                    tableau[r][nextCol] = -1.0;
                    nextCol++; eCounter++;

                    allNames.Add($"a{aCounter}");
                    tableau[r][nextCol] = 1.0;
                    basis[r] = nextCol;
                    artificialColumns.Add(nextCol);
                    nextCol++; aCounter++;
                }
                else // "="
                {
                    allNames.Add($"a{aCounter}");
                    tableau[r][nextCol] = 1.0;
                    basis[r] = nextCol;
                    artificialColumns.Add(nextCol);
                    nextCol++; aCounter++;
                }
            }

            // ---------- Step 4: objective row. ----------
            // The pivot rules below always drive toward a MAXIMUM. For a "min" model we
            // internally maximise the negated objective and flip signs back at the end
            // (dirSign), which is the standard trick -- without it, a "min" model would
            // silently be solved as if it were "max" and return the wrong vertex.
            double dirSign = model.IsMax ? 1.0 : -1.0;
            var internalObj = workingObj.Select(v => v * dirSign).ToArray();

            var objRow = new double[nTotal + 1];
            for (int c = 0; c < nWorking; c++) objRow[c] = -internalObj[c];
            foreach (var a in artificialColumns) objRow[a] = BigM;

            // Eliminate basic variables' coefficients from the objective row (row-reduce so obj row reads 0 under the basis)
            for (int r = 0; r < nCons; r++)
            {
                int b = basis[r];
                if (Math.Abs(objRow[b]) > Eps)
                {
                    double factor = objRow[b];
                    for (int c = 0; c <= nTotal; c++) objRow[c] -= factor * tableau[r][c];
                }
            }

            var result = new SimplexResult { Columns = columns };
            var iterations = new List<TableauSnapshot>();

            void Snapshot(string note)
            {
                iterations.Add(new TableauSnapshot
                {
                    IterationNumber = iterations.Count,
                    BasisNames = basis.Select(b => allNames[b]).ToList(),
                    Body = tableau.Select(row => (double[])row.Clone()).ToArray(),
                    ObjectiveRow = (double[])objRow.Clone(),
                    Note = note
                });
            }

            Snapshot("Initial tableau (canonical form)");

            int iteration = 0;
            while (iteration < MaxIterations)
            {
                // Entering column: most negative objective-row coefficient (standard max-simplex rule)
                int entering = -1;
                double mostNegative = -Eps;
                for (int c = 0; c < nTotal; c++)
                {
                    if (objRow[c] < mostNegative)
                    {
                        mostNegative = objRow[c];
                        entering = c;
                    }
                }

                if (entering == -1) break; // optimal

                // Ratio test: smallest non-negative RHS/column ratio, only over strictly positive entries
                int leaving = -1;
                double bestRatio = double.PositiveInfinity;
                for (int r = 0; r < nCons; r++)
                {
                    if (tableau[r][entering] > Eps)
                    {
                        double ratio = tableau[r][nTotal] / tableau[r][entering];
                        if (ratio < bestRatio - Eps)
                        {
                            bestRatio = ratio;
                            leaving = r;
                        }
                    }
                }

                if (leaving == -1)
                {
                    result.Unbounded = true;
                    break;
                }

                // Pivot
                double pivotVal = tableau[leaving][entering];
                for (int c = 0; c <= nTotal; c++) tableau[leaving][c] /= pivotVal;

                for (int r = 0; r < nCons; r++)
                {
                    if (r == leaving) continue;
                    double factor = tableau[r][entering];
                    if (Math.Abs(factor) < Eps) continue;
                    for (int c = 0; c <= nTotal; c++) tableau[r][c] -= factor * tableau[leaving][c];
                }

                double objFactor = objRow[entering];
                if (Math.Abs(objFactor) > Eps)
                    for (int c = 0; c <= nTotal; c++) objRow[c] -= objFactor * tableau[leaving][c];

                basis[leaving] = entering;
                iteration++;
                Snapshot($"Pivot on column '{allNames[entering]}', row {leaving + 1} (entering={allNames[entering]})");

                if (result.Unbounded) break;
            }

            result.Iterations = iterations;
            result.BasisColumnIndex = basis.ToList();
            result.AllColumnNames = allNames;

            // ---------- Step 5: feasibility check -- any artificial variable left in the basis with value > 0 ----------
            for (int r = 0; r < nCons; r++)
            {
                if (artificialColumns.Contains(basis[r]) && tableau[r][nTotal] > 1e-6)
                {
                    result.Infeasible = true;
                }
            }

            if (result.Unbounded || result.Infeasible)
                return result;

            // ---------- Step 6: read off the working-variable solution, then fold back to original variables ----------
            var workingValues = new double[nTotal];
            for (int r = 0; r < nCons; r++) workingValues[basis[r]] = tableau[r][nTotal];

            var originalValues = new double[model.VariableCount];
            for (int c = 0; c < nWorking; c++)
                originalValues[columns[c].OriginalVariableIndex] += columns[c].Multiplier * workingValues[c];

            for (int i = 0; i < model.VariableCount; i++)
                result.OriginalSolution[$"x{i + 1}"] = Math.Round(originalValues[i], 3);

            double objValue = 0.0;
            for (int i = 0; i < model.VariableCount; i++)
                objValue += model.ObjectiveCoefficients[i] * originalValues[i];
            result.ObjectiveValue = Math.Round(objValue, 3);

            // ---------- Step 7: shadow prices -- objective-row value under each constraint's slack/surplus column ----------
            // These read the FINAL objRow (which is in terms of the internal max problem), then
            // multiply by dirSign to convert back to the original max/min problem's dual values
            // (verified against known textbook duals and independently against perturbation-based
            // dz*/db checks for <=, >= and = constraints, in both max and min models).
            //   <= row i, slack s_i          : dual_i = dirSign * objRow[s_i]
            //   >= row i, surplus e_i         : dual_i = dirSign * -objRow[e_i]
            //   =  row i, artificial a_i      : dual_i = dirSign * (objRow[a_i] - BigM)
            int col = nWorking;
            foreach (var (_, relation, _) in rows.Take(model.Constraints.Count)) // only report for the model's real constraints, not the auto 'bin' bounds
            {
                if (relation == "<=")
                {
                    result.ShadowPrices.Add(Math.Round(dirSign * objRow[col], 3));
                    col += 1;
                }
                else if (relation == ">=")
                {
                    result.ShadowPrices.Add(Math.Round(dirSign * -objRow[col], 3));
                    col += 2;
                }
                else
                {
                    result.ShadowPrices.Add(Math.Round(dirSign * (objRow[col] - BigM), 3));
                    col += 1;
                }
            }

            return result;
        }
    }
}
