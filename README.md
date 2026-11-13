# RoboticArmSimulator

A 3D robotic-arm simulator built with C# and WPF on .NET 10. The application uses the built-in WPF 3D stack (`Viewport3D`, `MeshGeometry3D`, and procedural meshes) without external 3D libraries.

## Features

- Procedural 3D robot scene with base, links, joints, wrist, gripper, conveyor, pickup zone, and drop zone.
- Hierarchical forward kinematics (FK).
- 3D damped-least-squares inverse kinematics with position and optional orientation solving.
- Configurable joint limits, including full base yaw from `-180°` to `+180°`.
- Four-finger gripper with open/close control and wrist roll.
- Wrist orientation axes and grip, cut, and weld tool visualizations.
- Manual object placement, pickup, attachment, and release.
- Automatic multi-object pick-and-place workflow.
- Safe-height automatic transport path for obstacle avoidance.
- Transparent blue plexiglass-style conveyor surface.
- Mouse-controlled camera orbit, zoom, and viewport interaction.
- Timestamped keyframe recording, trajectory playback, and JSON import/export.

## Obstacle-avoidance test

The **Test obstacle avoidance** button starts a separate test workflow:

- Obstacles appear only while the test is active.
- Each test generates a different random layout.
- Obstacles are approximately the size of grasped objects (`0.32 × 0.32 × 0.32`).
- Obstacles are placed in the robot's working path, outside the conveyor surface.
- The robot calculates a safe path above and around the generated obstacles.
- One object is picked up, transported to the drop zone, released, and returned home.
- Test poses are recorded as keyframes.
- Obstacles are removed when the test finishes or is stopped.
- **Test speed** can be adjusted from `1%` to `100%` for controlled movement.

## Requirements

- Windows 10/11.
- Visual Studio 2026 or newer.
- .NET 10 SDK.
- Visual Studio Windows desktop/WPF workload.

## Running

1. Open `RoboticArm.slnx` in Visual Studio.
2. Select the `RoboticArm` project.
3. Build and run the project.
4. Use the controls in the right-side panel.

### Manual IK

Enter target position and optional orientation values in the **IK (3D)** section and select **Solve 6D IK**. The solver respects the configured joint limits.

### Automatic pick and place

Use **Start operation** to process the conveyor objects. Use **Stop Auto** to stop the workflow and **Reset Auto Objects** to restore the initial scene and object positions.

### Keyframes

Use **Record Keyframe** to save the current joint pose. Use **Play Trajectory** and **Stop Trajectory** for playback. Keyframes can be cleared or persisted using **Export Keyframes** and **Import Keyframes**.

## Project structure

MVVM layout: the view is a thin shell, all state and behaviour live in the view model and services, and the 3D scene is rendered from view-model events.

- `MainWindow.xaml` - bindings, commands, and viewport host only.
- `MainWindow.xaml.cs` - composition root (view model, camera, renderer) plus viewport mouse handling for IK target selection.
- `ViewModels/MainViewModel.cs` - joint state, commands, timers, IK targets, object status, and automation wiring.
- `Models/RobotArmModel.cs` - robot state, FK, IK entry points, joint limits, and poses.
- `Models/AutomationItem.cs`, `Models/Obstacle.cs`, `Models/Keyframe.cs`, `Models/ToolMode.cs`, `Models/AutomationPhase.cs` - scene and workflow models.
- `Services/AutomationController.cs` - automatic pick-and-place state machine and transport path.
- `Services/KeyframePlayer.cs` - keyframe recording and Hermite trajectory playback.
- `Services/IDialogService.cs`, `Services/WpfDialogService.cs` - dialog abstraction and WPF implementation.
- `Mvvm/ObservableObject.cs`, `Mvvm/RelayCommand.cs` - `INotifyPropertyChanged` base and `ICommand`.
- `Rendering/RobotSceneRenderer.cs` - builds and updates the `Viewport3D` scene from view-model events.
- `Rendering/MeshBuilder.cs` - procedural mesh helpers.
- `Views/CameraController.cs` - orbit and zoom camera.
- `Kinematics/ForwardKinematics.cs` - rotation, translation, and planar kinematics helpers.
- `Kinematics/InverseKinematics6D.cs` - damped-least-squares 6D IK solver.

## Build

The project targets `net10.0-windows` and uses nullable reference types and implicit usings. Build it from Visual Studio with **Build > Build Solution**.

Visual Studio, .NET build output, and local publish files are excluded through `.gitignore`.
