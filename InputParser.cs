using System.Text.RegularExpressions;

namespace LPSolverPractice
{
    /// <summary>
    /// Reads the plain-text input file format described in the project brief:
    ///
    ///   line 1      : max|min  &lt;signed obj coeff&gt; &lt;signed obj coeff&gt; ...
    ///   line 2..n   : &lt;signed coeff&gt; ... &lt;relation&gt;&lt;rhs&gt;      (one per constraint)
    ///   last line   : sign restriction per variable ( +, -, urs, int, bin )
    ///
    /// Example:
    ///   max +2 +3 +3 +5 +2 +4
    ///   +11 +8 +6 +14 +10 +10 <=40
    ///   bin bin bin bin bin bin
    /// </summary>
    public static class InputParser
    {
        // Matches a relation ( <=, >=, = ) glued directly to its rhs number, e.g. "<=40" or "=12.5"
        private static readonly Regex RelationRhs = new(@"^(<=|>=|=)(-?\d+(\.\d+)?)$");

        public static LPModel Parse(string path)
        {
            var lines = File.ReadAllLines(path)
                             .Select(l => l.Trim())
                             .Where(l => l.Length > 0)
                             .ToList();

            if (lines.Count < 3)
                throw new FormatException("Input file must contain an objective line, at least one constraint line, and a sign-restriction line.");

            var model = new LPModel();

            // ---- line 1 : objective function ----------------------------------------------------
            var objTokens = lines[0].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            var direction = objTokens[0].ToLowerInvariant();
            if (direction != "max" && direction != "min")
                throw new FormatException($"First token of line 1 must be 'max' or 'min', got '{objTokens[0]}'.");
            model.IsMax = direction == "max";

            for (int i = 1; i < objTokens.Length; i++)
                model.ObjectiveCoefficients.Add(ParseSignedNumber(objTokens[i], "objective function", i));

            int varCount = model.ObjectiveCoefficients.Count;
            if (varCount == 0)
                throw new FormatException("No decision variables found on line 1.");

            // ---- middle lines : constraints -------------------------------------------------------
            for (int lineIdx = 1; lineIdx < lines.Count - 1; lineIdx++)
            {
                var tokens = lines[lineIdx].Split(' ', StringSplitOptions.RemoveEmptyEntries);
                if (tokens.Length != varCount + 1)
                    throw new FormatException(
                        $"Constraint on line {lineIdx + 1} has {tokens.Length - 1} coefficient(s), expected {varCount}.");

                var constraint = new Constraint();
                for (int i = 0; i < varCount; i++)
                    constraint.Coefficients.Add(ParseSignedNumber(tokens[i], $"constraint on line {lineIdx + 1}", i));

                var last = tokens[varCount];
                var match = RelationRhs.Match(last);
                if (!match.Success)
                    throw new FormatException(
                        $"Could not parse relation/rhs '{last}' on line {lineIdx + 1}. Expected e.g. '<=40'.");

                constraint.Relation = match.Groups[1].Value;
                constraint.Rhs = double.Parse(match.Groups[2].Value);

                model.Constraints.Add(constraint);
            }

            if (model.Constraints.Count == 0)
                throw new FormatException("No constraints were found between the objective line and the sign-restriction line.");

            // ---- last line : sign restrictions ------------------------------------------------------
            var signTokens = lines[^1].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (signTokens.Length != varCount)
                throw new FormatException(
                    $"Sign restriction line has {signTokens.Length} entries, expected {varCount}.");

            var allowed = new HashSet<string> { "+", "-", "urs", "int", "bin" };
            foreach (var t in signTokens)
            {
                var norm = t.ToLowerInvariant();
                if (!allowed.Contains(norm))
                    throw new FormatException($"Unknown sign restriction '{t}'. Expected one of +, -, urs, int, bin.");
                model.SignRestrictions.Add(norm);
            }

            return model;
        }

        private static double ParseSignedNumber(string token, string context, int position)
        {
            if (token.Length < 2 || (token[0] != '+' && token[0] != '-'))
                throw new FormatException(
                    $"Coefficient #{position + 1} in {context} ('{token}') must start with + or -.");

            if (!double.TryParse(token, out var value))
                throw new FormatException($"Could not parse coefficient '{token}' in {context}.");

            return value;
        }
    }
}
