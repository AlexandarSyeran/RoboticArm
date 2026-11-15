using RoboticArm.Services;

namespace RoboticArm.Tests
{
    public sealed class NumericInputTests
    {
        [Theory]
        [InlineData("0.7", 0.7)]
        [InlineData("2.5", 2.5)]
        [InlineData("1000.5", 1000.5)]
        [InlineData("-1.25", -1.25)]
        public void TryParse_UsesInvariantCulture(string text, double expected)
        {
            Assert.True(NumericInput.TryParse(text, out double value));
            Assert.Equal(expected, value, 9);
        }

        [Theory]
        [InlineData("")]
        [InlineData("abc")]
        [InlineData("0,7")]
        [InlineData("1,000.5")]
        [InlineData("1.2.3")]
        public void TryParse_RejectsInvalidText(string text)
        {
            Assert.False(NumericInput.TryParse(text, out _));
        }

        [Fact]
        public void Format_UsesDotDecimalSeparator()
        {
            Assert.Equal("2.50", NumericInput.Format(2.5));
        }
    }
}
