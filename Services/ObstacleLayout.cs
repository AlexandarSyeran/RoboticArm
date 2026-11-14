using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace RoboticArm.Services
{
    // Builds the obstacle course for the avoidance test. Nothing enters the
    // conveyor belt field (|z| <= 1.2): every obstacle stands in the side
    // lanes or floats beside the belt at a set height. The route still meets
    // the side lanes, because crossing the base forces the arm to swing
    // around the base axis at radius >= 1.2 - so the tall barriers beside
    // the base make the arm weave left and right ("around"), while the
    // floating beams and pillars fill the side space at other heights.
    public static class ObstacleLayout
    {
        public static List<Obstacle> Generate(double reachX, int testNumber)
        {
            var random = new Random(testNumber * 7919);
            double flip = random.Next(2) == 0 ? 1.0 : -1.0;
            double barrierJitter = (random.NextDouble() - 0.5) * 0.3;
            double beamLift = (random.NextDouble() - 0.5) * 0.3;
            double pillarHeight = 1.3 + random.NextDouble() * 0.3;

            // Tall barriers in opposite side lanes, staggered along X: the
            // base swing has to weave around them. Tall enough that neither
            // ducking nor climbing over is an option. They sit at |z| = 1.6
            // so the outer band stays wide enough for the grown payload
            // envelope during re-plans.
            var barriers = new List<Obstacle>
            {
                new() { Center = new Point3D(1.5 + barrierJitter, 1.65, 1.6 * flip), HalfSize = new Vector3D(0.6, 1.55, 0.2) },
                new() { Center = new Point3D(-1.5 + barrierJitter, 1.65, -1.6 * flip), HalfSize = new Vector3D(0.6, 1.55, 0.2) },
            };

            // Floating beams beside the belt at mid height.
            var beams = new List<Obstacle>
            {
                new() { Center = new Point3D(2.4, 2.5 + beamLift, -1.6 * flip), HalfSize = new Vector3D(0.5, 0.3, 0.25) },
                new() { Center = new Point3D(-2.4, 2.5 - beamLift, 1.6 * flip), HalfSize = new Vector3D(0.5, 0.3, 0.25) },
            };

            // Floor pillars further out, opposite lane from the beams.
            var pillars = new List<Obstacle>
            {
                new() { Center = new Point3D(3.3, 0.1 + pillarHeight, 1.5 * flip), HalfSize = new Vector3D(0.22, pillarHeight, 0.2) },
                new() { Center = new Point3D(-3.3, 0.1 + pillarHeight, -1.5 * flip), HalfSize = new Vector3D(0.22, pillarHeight, 0.2) },
            };

            var obstacles = new List<Obstacle>();
            obstacles.AddRange(barriers);
            obstacles.AddRange(beams);
            obstacles.AddRange(pillars);
            return obstacles;
        }
    }
}
