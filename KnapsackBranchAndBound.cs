namespace LPSolverPractice
{
    public class KnapsackNode
    {
        public int Level { get; set; }
        public string Path { get; set; } = "";
        public string Note { get; set; } = "";
    }

    public class KnapsackResult
    {
        public bool Applicable { get; set; } = true;
        public string? NotApplicableReason { get; set; }
        public List<KnapsackNode> Nodes { get; set; } = new();
        public double BestValue { get; set; }
        public int[] BestSelection { get; set; } = Array.Empty<int>();
    }

    /// <summary>
    /// Branch & Bound Knapsack Algorithm -- specialised for the plain 0/1 knapsack
    /// (one maximisation objective, one &lt;= capacity constraint, every variable 'bin',
    /// non-negative profits/weights: exactly the shape of the brief's example IP).
    ///
    /// This is a DIFFERENT algorithm from Branch &amp; Bound Simplex: instead of calling the
    /// full simplex on every node, items are sorted once by profit/weight ratio and each
    /// node's bound is the classic fractional-knapsack (greedy) bound -- fill items in
    /// ratio order until the next one would overflow the capacity, then take a fractional
    /// slice of that next item. That bound is cheap to compute and, because it relaxes only
    /// the 0/1 restriction (not the capacity constraint), it is provably tighter here than
    /// the general LP-relaxation bound Branch &amp; Bound Simplex would compute.
    ///
    /// Depth-first with explicit backtracking: at each depth we branch on "include item"
    /// then "exclude item", logging every subproblem, every fathomed node and every
    /// backtrack step, and keep the best complete (integer-feasible) selection seen.
    /// </summary>
    public static class KnapsackBranchAndBound
    {
        public static KnapsackResult Solve(LPModel model)
        {
            var result = new KnapsackResult();

            if (!model.IsMax)
            {
                result.Applicable = false;
                result.NotApplicableReason = "Knapsack Branch & Bound expects a maximisation model.";
                return result;
            }
            if (model.Constraints.Count != 1)
            {
                result.Applicable = false;
                result.NotApplicableReason = $"expects exactly one capacity constraint, found {model.Constraints.Count}.";
                return result;
            }
            if (model.SignRestrictions.Count != model.VariableCount || model.SignRestrictions.Any(s => s != "bin"))
            {
                result.Applicable = false;
                result.NotApplicableReason = "expects every decision variable to be restricted 'bin'.";
                return result;
            }
            var con = model.Constraints[0];
            if (con.Relation != "<=")
            {
                result.Applicable = false;
                result.NotApplicableReason = "expects the single constraint to be a '<=' capacity limit.";
                return result;
            }
            if (model.ObjectiveCoefficients.Any(v => v < 0) || con.Coefficients.Any(v => v < 0))
            {
                result.Applicable = false;
                result.NotApplicableReason = "expects non-negative profits and weights.";
                return result;
            }

            int n = model.VariableCount;
            var profit = model.ObjectiveCoefficients.ToArray();
            var weight = con.Coefficients.ToArray();
            double capacity = con.Rhs;

            var order = Enumerable.Range(0, n).OrderByDescending(rank => profit[rank] / weight[rank]).ToArray();

            double Bound(int level, double curWeight, double curValue)
            {
                if (curWeight > capacity) return 0.0;
                double b = curValue;
                double w = curWeight;
                int j = level;
                while (j < n && w + weight[order[j]] <= capacity)
                {
                    w += weight[order[j]];
                    b += profit[order[j]];
                    j++;
                }
                if (j < n) b += (capacity - w) * profit[order[j]] / weight[order[j]];
                return b;
            }

            double bestValue = 0.0;
            int[] bestSel = new int[n];

            void Recurse(int level, double curWeight, double curValue, int[] sel)
            {
                if (level == n)
                {
                    result.Nodes.Add(new KnapsackNode { Level = level, Path = DescribeSel(sel, order, level), Note = $"Leaf: value = {curValue:0.###}" });
                    if (curValue > bestValue) { bestValue = curValue; bestSel = (int[])sel.Clone(); }
                    return;
                }

                int idx = order[level];
                double b = Bound(level, curWeight, curValue);
                result.Nodes.Add(new KnapsackNode
                {
                    Level = level,
                    Path = DescribeSel(sel, order, level),
                    Note = $"Sub-problem: decide x{idx + 1}. Fractional-relaxation bound = {b:0.###}"
                });

                if (b <= bestValue + 1e-9)
                {
                    result.Nodes.Add(new KnapsackNode { Level = level, Note = "FATHOM: bound does not exceed the current best candidate -- pruned." });
                    return;
                }

                if (curWeight + weight[idx] <= capacity)
                {
                    var selIn = (int[])sel.Clone(); selIn[idx] = 1;
                    Recurse(level + 1, curWeight + weight[idx], curValue + profit[idx], selIn);
                }
                else
                {
                    result.Nodes.Add(new KnapsackNode { Level = level, Note = $"Including x{idx + 1} would exceed capacity -- that branch is infeasible." });
                }

                var selOut = (int[])sel.Clone(); selOut[idx] = 0;
                Recurse(level + 1, curWeight, curValue, selOut);

                result.Nodes.Add(new KnapsackNode { Level = level, Note = $"Backtracking: both x{idx + 1}=1 and x{idx + 1}=0 explored at depth {level}." });
            }

            Recurse(0, 0.0, 0.0, new int[n]);

            result.BestValue = bestValue;
            result.BestSelection = bestSel;
            return result;
        }

        private static string DescribeSel(int[] sel, int[] order, int level)
        {
            var parts = new List<string>();
            for (int k = 0; k < level; k++)
            {
                int idx = order[k];
                parts.Add($"x{idx + 1}={sel[idx]}");
            }
            return string.Join(", ", parts);
        }

        public static string BuildReport(LPModel model, KnapsackResult result)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("=== BRANCH & BOUND KNAPSACK ALGORITHM ===");

            if (!result.Applicable)
            {
                sb.AppendLine("This model is not a plain 0/1 knapsack: " + result.NotApplicableReason);
                sb.AppendLine("Use the general Branch & Bound Simplex option instead.");
                return sb.ToString();
            }

            int n = model.VariableCount;
            var profit = model.ObjectiveCoefficients;
            var weight = model.Constraints[0].Coefficients;
            var order = Enumerable.Range(0, n).OrderByDescending(rank => profit[rank] / weight[rank]).ToList();

            sb.AppendLine("Items sorted by profit/weight ratio (the order branching explores them in):");
            foreach (var i in order)
                sb.AppendLine($"  x{i + 1}: profit={profit[i]:0.###}, weight={weight[i]:0.###}, ratio={(profit[i] / weight[i]):0.###}");
            sb.AppendLine($"Capacity = {model.Constraints[0].Rhs:0.###}");
            sb.AppendLine();

            sb.AppendLine("--- Search tree (every sub-problem, fathoming and backtrack step) ---");
            foreach (var node in result.Nodes)
            {
                if (!string.IsNullOrEmpty(node.Path))
                    sb.AppendLine($"[depth {node.Level}] fixed so far: {node.Path}  ->  {node.Note}");
                else
                    sb.AppendLine($"[depth {node.Level}] {node.Note}");
            }
            sb.AppendLine();

            sb.AppendLine("=== BEST CANDIDATE (Branch & Bound Knapsack) ===");
            sb.AppendLine($"z = {result.BestValue:0.000}");
            for (int i = 0; i < n; i++)
                sb.AppendLine($"  x{i + 1} = {result.BestSelection[i]}");

            return sb.ToString();
        }
    }
}
