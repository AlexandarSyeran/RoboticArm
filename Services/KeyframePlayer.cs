using RoboticArm.Models;

namespace RoboticArm.Services
{
    public sealed class KeyframePlayer
    {
        private double _time;

        public List<Keyframe> Keyframes { get; } = new();
        public bool IsPlaying { get; private set; }

        public void Record(double[] angles)
        {
            double time = Keyframes.Count == 0 ? 0 : Keyframes[^1].Time + 1.0;
            Keyframes.Add(new Keyframe { Time = time, Angles = (double[])angles.Clone() });
        }

        public bool Play()
        {
            if (Keyframes.Count < 2)
                return false;

            Keyframes.Sort((a, b) => a.Time.CompareTo(b.Time));
            _time = 0;
            IsPlaying = true;
            return true;
        }

        public void Stop() => IsPlaying = false;

        public bool Tick(double elapsedSeconds, double[] angles)
        {
            if (!IsPlaying)
                return false;

            if (Keyframes.Count < 2)
            {
                IsPlaying = false;
                return false;
            }

            _time += elapsedSeconds;
            double total = Keyframes[^1].Time;
            if (_time > total)
            {
                IsPlaying = false;
                return false;
            }

            int i1 = 0;
            for (int i = 0; i < Keyframes.Count - 1; i++)
            {
                if (_time >= Keyframes[i].Time && _time <= Keyframes[i + 1].Time)
                {
                    i1 = i;
                    break;
                }
            }

            int i0 = Math.Max(0, i1 - 1);
            int i2 = Math.Min(Keyframes.Count - 1, i1 + 1);
            int i3 = Math.Min(Keyframes.Count - 1, i1 + 2);

            var k0 = Keyframes[i0];
            var k1 = Keyframes[i1];
            var k2 = Keyframes[i2];
            var k3 = Keyframes[i3];

            double t0 = k1.Time;
            double t1 = k2.Time;
            double segmentDuration = Math.Max(1e-6, t1 - t0);
            double s = Math.Clamp((_time - t0) / segmentDuration, 0.0, 1.0);

            for (int j = 0; j < angles.Length; j++)
            {
                // Older files were recorded with four joints; missing values
                // fall back to zero so imported trajectories still play.
                double p0 = AngleAt(k0, j);
                double p1 = AngleAt(k1, j);
                double p2 = AngleAt(k2, j);
                double p3 = AngleAt(k3, j);

                double denom1 = Math.Max(1e-6, k2.Time - k0.Time);
                double m1 = (p2 - p0) / denom1;
                double denom2 = Math.Max(1e-6, k3.Time - k1.Time);
                double m2 = (p3 - p1) / denom2;

                double s2 = s * s;
                double s3 = s2 * s;
                double h00 = 2 * s3 - 3 * s2 + 1;
                double h10 = s3 - 2 * s2 + s;
                double h01 = -2 * s3 + 3 * s2;
                double h11 = s3 - s2;

                angles[j] = h00 * p1 + h10 * (m1 * segmentDuration) + h01 * p2 + h11 * (m2 * segmentDuration);
            }

            return true;
        }

        private static double AngleAt(Keyframe keyframe, int index)
            => index < keyframe.Angles.Length ? keyframe.Angles[index] : 0.0;

        public void Replace(IEnumerable<Keyframe> keyframes)
        {
            Keyframes.Clear();
            Keyframes.AddRange(keyframes);
        }
    }
}
