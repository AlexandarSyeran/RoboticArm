namespace RoboticArm.Models
{
    public sealed class Keyframe
    {
        public double Time { get; set; }
        public double[] Angles { get; set; } = Array.Empty<double>();
    }
}
