namespace LPSolverPractice
{
    /// <summary>
    /// Menu-driven entry point, matching the brief's "solve.exe" idea:
    ///   - load an input text file with the mathematical model
    ///   - pick an algorithm to solve it (Primal Simplex, Revised Simplex, Branch & Bound
    ///     Simplex, Cutting Plane, Branch & Bound Knapsack)
    ///   - run sensitivity analysis / duality on the last Primal Simplex solve
    ///   - export the last report (canonical form + all iterations + result) to a file
    ///
    /// _lastReportText always holds whatever was last shown on screen, so option 11 ("save
    /// to file") works uniformly no matter which algorithm or analysis produced it.
    /// _lastPrimalResult specifically holds the last Big-M Primal Simplex tableau, because
    /// that is what SensitivityAnalysis.cs and Duality.cs are built to read from.
    /// </summary>
    public static class Program
    {
        private static LPModel? _model;
        private static SimplexResult? _lastPrimalResult;
        private static string? _lastReportText;
        private static string? _loadedPath;

        public static void Main(string[] args)
        {
            Console.WriteLine("========================================");
            Console.WriteLine(" LPR381 Project");
            Console.WriteLine("========================================");

            if (args.Length > 0) TryLoad(args[0]);

            bool running = true;
            while (running)
            {
                PrintMenu();
                var choice = Console.ReadLine()?.Trim();
                switch (choice)
                {
                    case "1": LoadModelInteractive(); break;
                    case "2": DisplayCanonicalForm(); break;
                    case "3": SolvePrimalSimplex(); break;
                    case "4": SolveRevisedSimplex(); break;
                    case "5": SolveBranchAndBoundSimplex(); break;
                    case "6": SolveCuttingPlane(); break;
                    case "7": SolveKnapsackBranchAndBound(); break;
                    case "8": ShowSolution(); break;
                    case "9": SensitivityMenu(); break;
                    case "10": SolveDuality(); break;
                    case "11": SaveOutputFile(); break;
                    case "0": running = false; break;
                    default: Console.WriteLine("Unrecognised option.\n"); break;
                }
            }
        }

        private static void PrintMenu()
        {
            Console.WriteLine();
            Console.WriteLine($"Loaded model: {(_loadedPath ?? "(none)")}");
            Console.WriteLine("1.  Load input file");
            Console.WriteLine("2.  Display canonical form");
            Console.WriteLine("3.  Primal Simplex");
            Console.WriteLine("4.  Revised Primal Simplex");
            Console.WriteLine("5.  Branch & Bound Simplex");
            Console.WriteLine("6.  Cutting Plane");
            Console.WriteLine("7.  Branch & Bound Knapsack");
            Console.WriteLine("8.  Show last Primal Simplex solution / shadow prices");
            Console.WriteLine("9.  Sensitivity Analysis menu (uses the last Primal Simplex solve)");
            Console.WriteLine("10. Duality (build + solve the dual, check strong duality)");
            Console.WriteLine("11. Save last report to an output file");
            Console.WriteLine("0.  Exit");
            Console.Write("> ");
        }

        // ================= loading =================

        private static void LoadModelInteractive()
        {
            Console.Write("Path to input text file: ");
            var path = Console.ReadLine()?.Trim() ?? "";
            TryLoad(path);
        }

        private static void TryLoad(string path)
        {
            try
            {
                _model = InputParser.Parse(path);
                _loadedPath = path;
                _lastPrimalResult = null;
                _lastReportText = null;
                Console.WriteLine($"Loaded {_model.VariableCount} variable(s) and {_model.Constraints.Count} constraint(s).");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Could not load '{path}': {ex.Message}");
            }
        }

        private static bool EnsureModelLoaded()
        {
            if (_model != null) return true;
            Console.WriteLine("Load an input file first (option 1).");
            return false;
        }

        // ================= algorithms =================

        private static void DisplayCanonicalForm()
        {
            if (!EnsureModelLoaded()) return;
            var result = SimplexSolver.Solve(_model!);
            Console.WriteLine(OutputWriter.BuildReport(_model!, result).Split("=== TABLEAU")[0]);
        }

        private static void SolvePrimalSimplex()
        {
            if (!EnsureModelLoaded()) return;
            _lastPrimalResult = SimplexSolver.Solve(_model!);
            _lastReportText = OutputWriter.BuildReport(_model!, _lastPrimalResult);
            Console.WriteLine(_lastReportText);
        }

        private static void SolveRevisedSimplex()
        {
            if (!EnsureModelLoaded()) return;
            var result = RevisedSimplex.Solve(_model!);
            _lastReportText = RevisedSimplex.BuildReport(_model!, result);
            Console.WriteLine(_lastReportText);
        }

        private static void SolveBranchAndBoundSimplex()
        {
            if (!EnsureModelLoaded()) return;
            var bnb = BranchAndBoundSimplex.Solve(_model!);
            _lastReportText = BranchAndBoundSimplex.BuildReport(_model!, bnb);
            Console.WriteLine(_lastReportText);
        }

        private static void SolveCuttingPlane()
        {
            if (!EnsureModelLoaded()) return;
            var result = CuttingPlane.Solve(_model!);
            _lastReportText = CuttingPlane.BuildReport(_model!, result);
            Console.WriteLine(_lastReportText);
        }

        private static void SolveKnapsackBranchAndBound()
        {
            if (!EnsureModelLoaded()) return;
            var result = KnapsackBranchAndBound.Solve(_model!);
            _lastReportText = KnapsackBranchAndBound.BuildReport(_model!, result);
            Console.WriteLine(_lastReportText);
        }

        private static void SolveDuality()
        {
            if (!EnsureModelLoaded()) return;
            _lastPrimalResult ??= SimplexSolver.Solve(_model!);
            _lastReportText = Duality.BuildReport(_model!, _lastPrimalResult!);
            Console.WriteLine(_lastReportText);
        }

        private static void ShowSolution()
        {
            if (_lastPrimalResult == null)
            {
                Console.WriteLine("Nothing solved with Primal Simplex yet -- use option 3 first.");
                return;
            }
            if (_lastPrimalResult.Infeasible) { Console.WriteLine("Model is INFEASIBLE."); return; }
            if (_lastPrimalResult.Unbounded) { Console.WriteLine("Model is UNBOUNDED."); return; }

            Console.WriteLine($"z = {_lastPrimalResult.ObjectiveValue:0.000}");
            foreach (var kv in _lastPrimalResult.OriginalSolution)
                Console.WriteLine($"  {kv.Key} = {kv.Value:0.000}");
            Console.WriteLine("Shadow prices:");
            for (int i = 0; i < _lastPrimalResult.ShadowPrices.Count; i++)
                Console.WriteLine($"  constraint {i + 1}: {_lastPrimalResult.ShadowPrices[i]:0.000}");
        }

        private static void SaveOutputFile()
        {
            if (_lastReportText == null)
            {
                Console.WriteLine("Nothing to save yet -- solve or analyse something first (options 3-10).");
                return;
            }
            Console.Write("Output file path (e.g. output.txt): ");
            var path = Console.ReadLine()?.Trim() ?? "output.txt";
            File.WriteAllText(path, _lastReportText);
            Console.WriteLine($"Report written to {path}");
        }

        // ================= sensitivity analysis =================

        private static void SensitivityMenu()
        {
            if (!EnsureModelLoaded()) return;
            _lastPrimalResult ??= SimplexSolver.Solve(_model!);
            if (_lastPrimalResult.Infeasible || _lastPrimalResult.Unbounded)
            {
                Console.WriteLine("The last Primal Simplex solve is infeasible/unbounded -- sensitivity analysis needs an optimal solution.");
                return;
            }

            bool inMenu = true;
            while (inMenu)
            {
                Console.WriteLine();
                Console.WriteLine("--- Sensitivity Analysis ---");
                Console.WriteLine("1. Range of a non-basic variable's cost");
                Console.WriteLine("2. Apply a change to a non-basic variable's cost");
                Console.WriteLine("3. Range of a basic variable's cost");
                Console.WriteLine("4. Apply a change to a basic variable's cost");
                Console.WriteLine("5. Range of a constraint's RHS");
                Console.WriteLine("6. Apply a change to a constraint's RHS");
                Console.WriteLine("7. Range of a non-basic column's technological coefficient");
                Console.WriteLine("8. Apply a change to a non-basic column's technological coefficient");
                Console.WriteLine("9. Add a new activity to the optimal solution");
                Console.WriteLine("10. Add a new constraint to the optimal solution");
                Console.WriteLine("11. Display shadow prices");
                Console.WriteLine("0. Back to main menu");
                Console.Write("> ");
                var choice = Console.ReadLine()?.Trim();

                string? report = choice switch
                {
                    "1" => SensitivityAnalysis.RangeNonBasicVariable(_model!, _lastPrimalResult!, ReadVarIndex("Variable")),
                    "2" => SensitivityAnalysis.ApplyNonBasicVariableChange(_model!, ReadVarIndex("Variable"), ReadDouble("New objective coefficient")),
                    "3" => SensitivityAnalysis.RangeBasicVariable(_model!, _lastPrimalResult!, ReadVarIndex("Variable")),
                    "4" => SensitivityAnalysis.ApplyBasicVariableChange(_model!, ReadVarIndex("Variable"), ReadDouble("New objective coefficient")),
                    "5" => SensitivityAnalysis.RangeRhs(_model!, _lastPrimalResult!, ReadConstraintIndex()),
                    "6" => SensitivityAnalysis.ApplyRhsChange(_model!, ReadConstraintIndex(), ReadDouble("New RHS")),
                    "7" => SensitivityAnalysis.RangeNonBasicColumnCoefficient(_model!, _lastPrimalResult!, ReadVarIndex("Variable"), ReadConstraintIndex()),
                    "8" => SensitivityAnalysis.ApplyColumnCoefficientChange(_model!, ReadVarIndex("Variable"), ReadConstraintIndex(), ReadDouble("New coefficient")),
                    "9" => ReadAndAddNewActivity(),
                    "10" => ReadAndAddNewConstraint(),
                    "11" => SensitivityAnalysis.DisplayShadowPrices(_model!, _lastPrimalResult!),
                    "0" => null,
                    _ => "Unrecognised option."
                };

                if (choice == "0") { inMenu = false; continue; }
                if (report != null)
                {
                    _lastReportText = report;
                    Console.WriteLine(report);
                }
            }
        }

        private static string ReadAndAddNewActivity()
        {
            double objCoeff = ReadDouble("New activity's objective coefficient");
            var coeffs = new List<double>();
            for (int i = 0; i < _model!.Constraints.Count; i++)
                coeffs.Add(ReadDouble($"  coefficient in constraint {i + 1}"));
            Console.Write("Sign restriction for the new variable (+, -, urs, int, bin) [default +]: ");
            var sign = Console.ReadLine()?.Trim().ToLowerInvariant();
            if (string.IsNullOrEmpty(sign)) sign = "+";
            return SensitivityAnalysis.AddNewActivity(_model!, _lastPrimalResult!, objCoeff, coeffs, sign);
        }

        private static string ReadAndAddNewConstraint()
        {
            var coeffs = new List<double>();
            for (int i = 0; i < _model!.VariableCount; i++)
                coeffs.Add(ReadDouble($"  coefficient for x{i + 1}"));
            Console.Write("Relation (<=, >=, =): ");
            var relation = Console.ReadLine()?.Trim() ?? "<=";
            double rhs = ReadDouble("RHS");
            return SensitivityAnalysis.AddNewConstraint(_model!, _lastPrimalResult!, coeffs, relation, rhs);
        }

        private static int ReadVarIndex(string label)
        {
            Console.Write($"{label} number (e.g. 1 for x1): ");
            int.TryParse(Console.ReadLine()?.Trim(), out var n);
            return n - 1;
        }

        private static int ReadConstraintIndex()
        {
            Console.Write("Constraint number (e.g. 1 for the first constraint): ");
            int.TryParse(Console.ReadLine()?.Trim(), out var n);
            return n - 1;
        }

        private static double ReadDouble(string label)
        {
            Console.Write($"{label}: ");
            double.TryParse(Console.ReadLine()?.Trim(), out var v);
            return v;
        }
    }
}
