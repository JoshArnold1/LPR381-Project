# LPR381 Project — LP/IP Solver

A menu-driven console program (`solve.exe`) that solves Linear Programming and Integer
Programming models, then lets you analyse how changes to the model affect the optimal
solution. Built for the LPR381 Programming project.

## Building

```bash
dotnet build
```

Produces `bin/Debug/net8.0/solve.exe`.

## Running

```bash
dotnet run
```

or run `solve.exe` directly. You can optionally pass an input file path as the first
argument to load it on startup:

```bash
solve.exe Examples/knapsack_from_brief.txt
```

The program then presents a menu:

```
1.  Load input file
2.  Display canonical form
3.  Primal Simplex
4.  Revised Primal Simplex
5.  Branch & Bound Simplex
6.  Cutting Plane
7.  Branch & Bound Knapsack
8.  Show last Primal Simplex solution / shadow prices
9.  Sensitivity Analysis menu (uses the last Primal Simplex solve)
10. Duality (build + solve the dual, check strong duality)
11. Save last report to an output file
0.  Exit
```

## Input file format

```
max|min <signed obj coeff> <signed obj coeff> ...
<signed coeff> ... <relation><rhs>      (one line per constraint)
<sign restriction> ...                  (one per decision variable: +, -, urs, int, bin)
```

Example (0/1 knapsack from the brief):

```
max +2 +3 +3 +5 +2 +4
+11 +8 +6 +14 +10 +10 <=40
bin bin bin bin bin bin
```

Sample inputs are in [`Examples/`](Examples/).

## Output

Every solve/analysis option prints a report (canonical form, all tableau iterations,
final result, shadow prices) to the console; option 11 saves the last report shown to a
text file. All values are rounded to 3 decimal places.

## Algorithms implemented

| File | Algorithm |
|---|---|
| [`SimplexSolver.cs`](SimplexSolver.cs) | Primal Simplex (Big-M method) |
| [`RevisedSimplex.cs`](RevisedSimplex.cs) | Revised Primal Simplex (product form of the inverse, price-out) |
| [`BranchAndBoundSimplex.cs`](BranchAndBoundSimplex.cs) | Branch & Bound Simplex, with backtracking and fathoming |
| [`CuttingPlane.cs`](CuttingPlane.cs) | Cutting Plane (Gomory cuts + dual simplex) |
| [`KnapsackBranchAndBound.cs`](KnapsackBranchAndBound.cs) | Branch & Bound Knapsack (0/1 knapsack, greedy bound) |
| [`SensitivityAnalysis.cs`](SensitivityAnalysis.cs) | Sensitivity analysis (ranging and applying changes to costs, RHS, coefficients, new activities/constraints, shadow prices) |
| [`Duality.cs`](Duality.cs) | Builds and solves the dual model, checks strong/weak duality |

Supporting files: [`InputParser.cs`](InputParser.cs) (reads the input format above),
[`OutputWriter.cs`](OutputWriter.cs) (formats reports), [`LPModel.cs`](LPModel.cs)
(in-memory model), [`StandardFormBuilder.cs`](StandardFormBuilder.cs) (shared
standard-form construction used by the Revised Simplex and Cutting Plane algorithms).

## Known limitations

- Sensitivity analysis (ranging and column-coefficient operations) only supports
  decision variables with sign restriction `+`, `int` or `bin`. Variables restricted
  `-` or `urs` are expanded into multiple working columns internally and are not
  supported by those specific operations.
- Branch & Bound Knapsack only applies to a plain 0/1 knapsack model: one
  maximisation objective, exactly one `<=` capacity constraint, every variable `bin`,
  and non-negative profits/weights. Any other shape falls back to Branch & Bound
  Simplex.
- Non-linear problem solving (bonus criteria) is not implemented.
