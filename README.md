# RoboticArmSimulator

3D robotic arm simulator built with C# and WPF on .NET 10.

## Features

- Built-in WPF 3D rendering with `Viewport3D` and procedural meshes.
- Folded initial robot pose with configurable joint limits.
- Base yaw limited to -90° through +90°.
- 3D position and orientation inverse kinematics.
- Four-finger gripper with wrist roll and visual orientation axes.
- Pick-and-attach behavior for scene objects.
- Automatic multi-object pick-and-place workflow.
- Configurable arrival and action delays.
- Keyframe recording, timestamped trajectory playback, and JSON import/export.
- Cut and weld tool visualizations.

## Requirements

- Visual Studio 2026 or newer.
- .NET 10 SDK.
- Windows desktop/WPF workload.

## Running

Open `RoboticArm.slnx` in Visual Studio and run the `RoboticArm` project.

Use **Start operation** in the **Automatic Pick and Place** section to start the automated workflow. Use **Reset Auto Objects** to restore the initial folded pose and scene objects.
