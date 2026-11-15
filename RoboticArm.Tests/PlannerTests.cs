using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace RoboticArm.Tests
{
    public sealed class PlannerTests
    {
        [Fact]
        public void Plan_AnchorsEndpointsWithoutObstacles()
        {
            var start = new Point3D(3, 1, 0);
            var end = new Point3D(-3, 1, 0);

            var plan = TransportPathPlanner.Plan(
                start, end, Array.Empty<Obstacle>(), new Vector3D(), 0.24, 1.0, 2.6);

            Assert.NotEqual("unreachable", plan.Mode);
            Assert.True(plan.Points.Count >= 2);
            Assert.Equal(start.X, plan.Points[0].X, 9);
            Assert.Equal(start.Z, plan.Points[0].Z, 9);
            Assert.Equal(end, plan.Points[^1]);
            Assert.Equal("straight", plan.Mode);
        }

        [Fact]
        public void Plan_ReportsUnreachableWhenObstacleSealsTheWorkspace()
        {
            var blocker = new Obstacle
            {
                Center = new Point3D(0, 1.2, 0),
                HalfSize = new Vector3D(10, 10, 10)
            };

            var plan = TransportPathPlanner.Plan(
                new Point3D(3, 1, 0),
                new Point3D(-3, 1, 0),
                new[] { blocker },
                new Vector3D(),
                payloadHalf: 0.24,
                lowLevel: 1.0,
                highLevel: 2.6);

            Assert.Equal("unreachable", plan.Mode);
        }
    }

    public sealed class RadarScannerTests
    {
        [Fact]
        public void Scan_FiltersZoneAndSortsByDistance()
        {
            var near = new AutomationItem { Position = new Point3D(0.1, 0.5, 0.1) };
            var far = new AutomationItem { Position = new Point3D(0.7, 0.5, 0.0) };
            var outside = new AutomationItem { Position = new Point3D(1.5, 0.5, 0.0) };
            var attached = new AutomationItem { Position = new Point3D(0.2, 0.5, 0.2), Attached = true };
            var completed = new AutomationItem { Position = new Point3D(0.3, 0.5, 0.3), Completed = true };
            var low = new AutomationItem { Position = new Point3D(0.2, 0.1, 0.2) };

            var scanner = new RadarScanner(
                new[] { far, outside, near, attached, completed, low },
                new Point3D(0, 0.3, 0));

            var found = scanner.Scan();

            Assert.Equal(2, found.Count);
            Assert.Same(near, found[0].Item);
            Assert.Same(far, found[1].Item);
        }
    }

    public sealed class OperationCoordinatorTests
    {
        [Fact]
        public void OnlyOneOperationCanRunAtATime()
        {
            var coordinator = new OperationCoordinator();

            Assert.True(coordinator.TryBegin(RobotOperation.Automation));
            Assert.False(coordinator.TryBegin(RobotOperation.Cutting));
            Assert.True(coordinator.Is(RobotOperation.Automation));

            coordinator.End(RobotOperation.Cutting);
            Assert.True(coordinator.Is(RobotOperation.Automation));

            coordinator.End(RobotOperation.Automation);
            Assert.True(coordinator.IsIdle);
            Assert.True(coordinator.TryBegin(RobotOperation.Wave));
        }

        [Fact]
        public void StopAll_ReturnsToIdleAndRaisesChange()
        {
            var coordinator = new OperationCoordinator();
            int changes = 0;
            coordinator.Changed += () => changes++;

            coordinator.TryBegin(RobotOperation.WristRotate);
            coordinator.StopAll();

            Assert.True(coordinator.IsIdle);
            Assert.Equal(2, changes);
        }
    }
}
