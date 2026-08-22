namespace LPSolverPractice
{
    /// <summary>
    /// One constraint row: a set of technological coefficients, a relation
    /// (&lt;=, &gt;= or =) and a right-hand-side value.
    /// </summary>
    public class Constraint
    {
        public List<double> Coefficients { get; set; } = new();
        public string Relation { get; set; } = "<="; // "<=", ">=" or "="
        public double Rhs { get; set; }

        public override string ToString()
        {
            var parts = Coefficients.Select(c => (c >= 0 ? "+" : "") + c.ToString("0.###"));
            return string.Join(" ", parts) + " " + Relation + " " + Rhs.ToString("0.###");
        }
    }

    /// <summary>
    /// In-memory representation of a Linear / Integer Programming model, exactly as
    /// described by the "Input Text File Criteria" section of the project brief:
    ///   line 1  -> max/min + signed objective coefficients
    ///   line(s) -> one per constraint: signed coefficients, relation, rhs
    ///   last line -> sign restrictions ( +, -, urs, int, bin ) per variable
    /// </summary>
    public class LPModel
    {
        public bool IsMax { get; set; }
        public List<double> ObjectiveCoefficients { get; set; } = new();
        public List<Constraint> Constraints { get; set; } = new();

        // One entry per decision variable: "+", "-", "urs", "int" or "bin"
        public List<string> SignRestrictions { get; set; } = new();

        public int VariableCount => ObjectiveCoefficients.Count;

        public List<string> VariableNames =>
            Enumerable.Range(1, VariableCount).Select(i => $"x{i}").ToList();

        public void Print(TextWriter writer)
        {
            writer.WriteLine((IsMax ? "max " : "min ") +
                string.Join(" ", ObjectiveCoefficients.Select(FormatSigned)));

            foreach (var c in Constraints)
            {
                writer.WriteLine(string.Join(" ", c.Coefficients.Select(FormatSigned)) +
                    " " + c.Relation + " " + c.Rhs.ToString("0.###"));
            }

            writer.WriteLine(string.Join(" ", SignRestrictions));
        }

        public static string FormatSigned(double v) =>
            (v >= 0 ? "+" : "") + Math.Round(v, 3).ToString("0.###");
    }
}
