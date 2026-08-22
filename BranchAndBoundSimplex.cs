namespace LPSolverPractice
{
    /// <summary>One node of the Branch & Bound search tree.</summary>
    public class BnBNode
    {
        public int Id { get; set; }
        public int ParentId { get; set; }
        public string BranchDescription { get; set; } = "";
        public LPModel Model { get; set; } = new();
        public SimplexResult? Result { get; set; }
        public string FathomReason { get; set; } = "";
        public bool IsCandidate { get; set; }
    }

    public class BranchAndBoundResult
    {
        public List<BnBNode> Nodes { get; set; } = new();
        public bool Found { get; set; }
        public double BestObjective { get; set; }
        public Dictionary<string, double> BestSolution { get; set; } = new();
        public int BestNodeId { get; set; } = -1;
    }

    /// <summary>
    /// Branch & Bound Simplex Algorithm.
    ///
    /// Every node's LP relaxation is solved by calling the already-working SimplexSolver
    /// (Big-M Primal Simplex) on a CLONE of the model with one extra bound constraint
    /// appended -- e.g. branching on a fractional x3 = 2.4 creates two children with
    /// "x3 &lt;= 2" and "x3 &gt;= 3" added as ordinary constraint rows. That is enough to
    /// reproduce the whole B&amp;B tree without touching the simplex internals at all: each
    /// node prints its own canonical form and full tableau iteration history via
    /// OutputWriter, exactly like solving that node's LP stand-alone would.
    ///
    /// Traversal is depth-first recursion (=explicit backtracking: after both children of a
    /// node have been fully explored -- fathomed or exhausted -- control returns to the
    /// parent and continues with its sibling branch). Fathoming rules implemented:
    ///   - infeasible relaxation            -> fathom
    ///   - unbounded relaxation             -> fathom (cannot usefully branch further)
    ///   - bound no better than incumbent   -> fathom by bound
    ///   - integer-feasible                 -> candidate solution (fathom by integrality)
    /// </summary>
    public static class BranchAndBoundSimplex
    {
        public static BranchAndBoundResult Solve(LPModel model)
        {
            var intIdx = new List<int>();
            for (int i = 0; i < model.VariableCount; i++)
                if (model.SignRestrictions.Count > i && (model.SignRestrictions[i] == "int" || model.SignRestrictions[i] == "bin"))
                    intIdx.Add(i);

            var overall = new BranchAndBoundResult();
            int nodeCounter = 0;
            double? bestVal = null;
            var bestSol = new Dictionary<string, double>();
            int bestNodeId = -1;

            void Recurse(LPModel nodeModel, int parentId, string branchDesc)
            {
                int myId = nodeCounter++;
                var node = new BnBNode { Id = myId, ParentId = parentId, BranchDescription = branchDesc, Model = nodeModel };
                overall.Nodes.Add(node);

                SimplexResult result;
                try { result = SimplexSolver.Solve(nodeModel); }
                catch (Exception ex) { node.FathomReason = $"Could not solve this node's relaxation: {ex.Message}"; return; }
                node.Result = result;

                if (result.Infeasible) { node.FathomReason = "Infeasible relaxation -- fathomed."; return; }
                if (result.Unbounded) { node.FathomReason = "Unbounded relaxation -- fathomed (cannot branch usefully further)."; return; }

                if (bestVal.HasValue)
                {
                    bool worseOrEqual = model.IsMax
                        ? result.ObjectiveValue <= bestVal.Value + 1e-6
                        : result.ObjectiveValue >= bestVal.Value - 1e-6;
                    if (worseOrEqual)
                    {
                        node.FathomReason =
                            $"Bound z={result.ObjectiveValue:0.000} does not improve on incumbent z={bestVal.Value:0.000} -- fathomed by bound.";
                        return;
                    }
                }

                int fracVar = -1;
                double fracVal = 0.0;
                foreach (var vidx in intIdx)
                {
                    double v = result.OriginalSolution.TryGetValue($"x{vidx + 1}", out var vv) ? vv : 0.0;
                    if (Math.Abs(v - Math.Round(v)) > 1e-6) { fracVar = vidx; fracVal = v; break; }
                }

                if (fracVar == -1)
                {
                    node.IsCandidate = true;
                    node.FathomReason = "All integer/binary variables are integer-valued -- candidate solution.";
                    bestVal = result.ObjectiveValue;
                    bestSol = new Dictionary<string, double>(result.OriginalSolution);
                    bestNodeId = myId;
                    return;
                }

                double floorV = Math.Floor(fracVal + 1e-9);
                double ceilV = Math.Ceiling(fracVal - 1e-9);

                node.FathomReason = $"Branching on fractional x{fracVar + 1} = {fracVal:0.###}.";

                var childLe = CloneWithBound(nodeModel, fracVar, "<=", floorV);
                Recurse(childLe, myId, $"x{fracVar + 1} <= {floorV:0.###}");

                var childGe = CloneWithBound(nodeModel, fracVar, ">=", ceilV);
                Recurse(childGe, myId, $"x{fracVar + 1} >= {ceilV:0.###}");

                node.FathomReason += " Both children explored -- backtracking to parent.";
            }

            Recurse(CloneModel(model), -1, "root (LP relaxation, no extra branching bounds)");

            overall.Found = bestVal.HasValue;
            if (overall.Found)
            {
                overall.BestObjective = bestVal!.Value;
                overall.BestSolution = bestSol;
                overall.BestNodeId = bestNodeId;
            }
            return overall;
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

        private static LPModel CloneWithBound(LPModel model, int varIndex, string relation, double rhs)
        {
            var clone = CloneModel(model);
            var row = new double[model.VariableCount];
            row[varIndex] = 1.0;
            clone.Constraints.Add(new Constraint { Coefficients = row.ToList(), Relation = relation, Rhs = rhs });
            return clone;
        }

        public static string BuildReport(LPModel rootModel, BranchAndBoundResult bnb)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("=== BRANCH & BOUND SIMPLEX ALGORITHM ===");
            var intVars = Enumerable.Range(0, rootModel.VariableCount)
                .Where(vi => rootModel.SignRestrictions.Count > vi &&
                            (rootModel.SignRestrictions[vi] == "int" || rootModel.SignRestrictions[vi] == "bin"))
                .Select(vi => $"x{vi + 1}");
            sb.AppendLine($"Root: {(rootModel.IsMax ? "max" : "min")} model, integer/binary variables: {string.Join(", ", intVars)}");
            sb.AppendLine($"Total nodes explored: {bnb.Nodes.Count}");
            sb.AppendLine();

            foreach (var node in bnb.Nodes)
            {
                sb.AppendLine($"########################################");
                sb.AppendLine($"# NODE #{node.Id}  (parent: {(node.ParentId == -1 ? "none / root" : "#" + node.ParentId)})");
                sb.AppendLine($"# Branch constraint added: {node.BranchDescription}");
                sb.AppendLine($"########################################");
                if (node.Result != null)
                    sb.AppendLine(OutputWriter.BuildReport(node.Model, node.Result));
                sb.AppendLine("--> " + node.FathomReason);
                sb.AppendLine();
            }

            sb.AppendLine("=== BEST CANDIDATE (Branch & Bound Simplex) ===");
            if (!bnb.Found)
            {
                sb.AppendLine("No integer-feasible solution exists -- the integer program is infeasible.");
            }
            else
            {
                sb.AppendLine($"Found at node #{bnb.BestNodeId}");
                sb.AppendLine($"z = {bnb.BestObjective:0.000}");
                foreach (var kv in bnb.BestSolution)
                    sb.AppendLine($"  {kv.Key} = {kv.Value:0.000}");
            }

            return sb.ToString();
        }
    }
}
