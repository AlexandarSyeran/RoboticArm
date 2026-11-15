using System.Globalization;

namespace RoboticArm.Services
{
    public static class NumericInput
    {
        // Thousands separators are deliberately excluded: in a comma-decimal
        // locale "0,7" would otherwise parse as 7 instead of failing.
        private const NumberStyles Style = NumberStyles.Float;

        public static bool TryParse(string? text, out double value)
            => double.TryParse(text, Style, CultureInfo.InvariantCulture, out value);

        public static string Format(double value, string format = "F2")
            => value.ToString(format, CultureInfo.InvariantCulture);
    }
}
