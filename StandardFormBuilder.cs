namespace LPSolverPractice
{
    /// <summary>
    /// The full standard/canonical form of an LPModel as flat arrays: working columns
    /// (sign-restricted variables expanded, exactly like SimplexSolver's Step 1-3),
    /// plus slack/surplus/artificial columns added on top (Step 3-4 in SimplexSolver.cs).
    ///
    /// This is deliberately a *copy* of the setup logic already written and working in
    /// SimplexSolver.cs, rather than a refactor of that file: SimplexSolver.cs is left
    /// untouched so the algorithm you already validated keeps behaving exactly as it did,
    /// while RevisedSimplex.cs and CuttingPlane.cs share this one building block instead
    /// of re-deriving it twice more, slightly differently, with new bugs each time.
    /// </summary>
    public class StandardForm
    {
        public List<WorkingColumn> Columns { get; set; } = new();
        public double[][] A { get; set; } = Array.Empty<double[]>(); // [row][col], nCons x nTotal
        public double[] B { get; set; } = Array.Empty<double>();     // nCons
        public double[] C { get; set; } = Array.Empty<double>();     // nTotal, ORIGINAL direction (not Big-M adjusted)
        public int[] Basis { get; set; } = Array.Empty<int>();       // initial basis, one column index per row
        public List<string> Names { get; set; } = new();             // nTotal column names
        public List<int> ArtificialColumns { get; set; } = new();
        public int WorkingColumnCount { get; set; }
        public int ConstraintCount { get; set; }                     // number of ORIGINAL model constraints (excludes auto bin<=1 rows)
        public double DirSign { get; set; }                          // +1 for max, -1 for min (internal-maximise trick)
    }

    public static class StandardFormBuilder
    {
        public static StandardForm Build(LPModel model)
        {
            // ---------- expand sign-restricted variables into working columns ----------
            var columns = new List<WorkingColumn>();
            for (int i = 0; i < model.VariableCount; i++)
            {
                var restriction = model.SignRestrictions.Count > i ? model.SignRestrictions[i] : "+";
                switch (restriction)
                {
                    case "-":
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}", OriginalVariableIndex = i, Multiplier = -1.0 });
                        break;
                    case "urs":
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}+", OriginalVariableIndex = i, Multiplier = 1.0 });
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}-", OriginalVariableIndex = i, Multiplier = -1.0 });
                        break;
                    default:
                        columns.Add(new WorkingColumn { Name = $"x{i + 1}", OriginalVariableIndex = i, Multiplier = 1.0 });
                        break;
                }
            }

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

            int originalConstraintCount = rows.Count;

            for (int i = 0; i < model.VariableCount; i++)
            {
                if (model.SignRestrictions.Count > i && model.SignRestrictions[i] == "bin")
                {
                    var row = new double[nWorking];
                    int colIdx = columns.FindIndex(wc => wc.OriginalVariableIndex == i);
                    row[colIdx] = 1.0;
                    rows.Add((row, "<=", 1.0));
                }
            }

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

            var allNames = columns.Select(wc => wc.Name).ToList();
            int slackCount = 0, surplusCount = 0, artificialCount = 0;
            foreach (var (_, relation, _) in rows)
            {
                if (relation == "<=") slackCount++;
                else if (relation == ">=") { surplusCount++; artificialCount++; }
                else artificialCount++;
            }

            int nTotal = nWorking + slackCount + surplusCount + artificialCount;
            var A = new double[nCons][];
            for (int r = 0; r < nCons; r++) A[r] = new double[nTotal];
            var b = new double[nCons];
            var basis = new int[nCons];
            var artificialColumns = new List<int>();
            int nextCol = nWorking, sCounter = 1, eCounter = 1, aCounter = 1;

            for (int r = 0; r < nCons; r++)
            {
                var (coeffs, relation, rhs) = rows[r];
                for (int c = 0; c < nWorking; c++) A[r][c] = coeffs[c];
                b[r] = rhs;

                if (relation == "<=")
                {
                    allNames.Add($"s{sCounter}");
                    A[r][nextCol] = 1.0;
                    basis[r] = nextCol;
                    nextCol++; sCounter++;
                }
                else if (relation == ">=")
                {
                    allNames.Add($"e{eCounter}");
                    A[r][nextCol] = -1.0;
                    nextCol++; eCounter++;

                    allNames.Add($"a{aCounter}");
                    A[r][nextCol] = 1.0;
                    basis[r] = nextCol;
                    artificialColumns.Add(nextCol);
                    nextCol++; aCounter++;
                }
                else
                {
                    allNames.Add($"a{aCounter}");
                    A[r][nextCol] = 1.0;
                    basis[r] = nextCol;
                    artificialColumns.Add(nextCol);
                    nextCol++; aCounter++;
                }
            }

            var cVec = new double[nTotal];
            for (int i = 0; i < nWorking; i++) cVec[i] = workingObj[i];

            return new StandardForm
            {
                Columns = columns,
                A = A,
                B = b,
                C = cVec,
                Basis = basis,
                Names = allNames,
                ArtificialColumns = artificialColumns,
                WorkingColumnCount = nWorking,
                ConstraintCount = originalConstraintCount,
                DirSign = model.IsMax ? 1.0 : -1.0
            };
        }
    }
}
