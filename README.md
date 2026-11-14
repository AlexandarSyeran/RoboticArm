# RoboticArmSimulator

A 3D robotic-arm simulator built with C# and WPF on .NET 10. The application uses the built-in WPF 3D stack (`Viewport3D`, `MeshGeometry3D`, and procedural meshes) without external 3D libraries.

## Features

- Procedural 3D robot scene with base, links, joints, wrist, gripper, conveyor, pickup zone, and drop zone.
- Five joints: base yaw, base elevation (`±45°`, lifts the whole arm), shoulder pitch, elbow pitch, and wrist pitch.
- Hierarchical forward kinematics (FK).
- 3D damped-least-squares inverse kinematics with position and optional orientation solving.
- Configurable joint limits, including full base yaw from `-180°` to `+180°`.
- Four-finger gripper with open/close control and wrist roll.
- Wrist orientation axes and grip, cut, weld tool visualizations, and an animated laser cutting workflow.
- Guided manual pick and place: the arm travels to an available object, grasps it, and later carries it to the drop zone.
- Automatic multi-object pick-and-place workflow.
- Grid-based transport path planner that routes the carried object over, under, and around obstacles.
- Transparent blue plexiglass-style conveyor surface.
- Built-in pickup-zone radar: the automation scans the zone and picks the nearest detected object; a bottom-left panel lists every detection with its dimensions and coordinates.
- Mouse-controlled camera orbit, zoom, and viewport interaction.
- Timestamped keyframe recording, trajectory playback, and JSON import/export.

## Obstacle-avoidance test

The **Test obstacle avoidance** button starts a separate test workflow:

- Obstacles appear only while the test is active.
- Each test generates a different random layout.
- All obstacles stand in the side lanes next to the conveyor belt - none of them lies in the belt field - at various heights: tall barriers, hanging beams, and pillars. The arm passes under, over, or around them.
- The carried object hangs off the tool, so its envelope swings with the wrist orientation along the route. The planner measures the actual per-position swing along the pose plan, re-plans with it, and keeps the least-clipping route.
- The robot plans the route on a height-layered grid, respecting the minimum carry radius and the arm's reach, then verifies every pose along it with inverse kinematics. When no collision-free route exists, the run stops with a clear message instead of attempting an impossible path.
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

### Manual pick and place

**Pick Object** sends the arm from wherever it stands to the pickup zone, approaches the nearest available object, and grasps it. The arm is then free for manual movement (sliders, IK target) or **Place Object**, which lifts the object to a safe height, carries it to the drop zone, lowers and releases it, and finally raises the arm off the placed object so the next **Pick Object** can start cleanly. **Close Gripper** still grabs an object that is already within reach of the gripper.

### Automatic pick and place

Use **Start operation** to process the conveyor objects. Use **Stop Auto** to stop the workflow and **Reset Auto Objects** to restore the initial scene and object positions. Objects are arranged in the drop zone like a pallet: a 2x2 grid, four objects per level, with the next four stacked on top, so they never land inside each other; the arrangement uses the internal drop scan and applies to both **Start operation** and manual **Place Object**. When the radar scan comes back empty, the run ends and a green message in the top-right corner reports that the transport operation finished; **Reset Auto Objects** clears it and restores the scene. The command panel uses the same dark palette as the radar readout.

### Laser cutting

Select **Cut** in the **Tool Mode** list and press **Activate Tool**. The arm visits the four objects on the pickup zone one by one, hovers above each one, lowers the tool, and burns a laser cut:

- a visible beam runs from the tool tip to the object while sparks mark the impact point;
- the cut plane follows the tool direction at that moment, so each object is split at the angle the arm actually holds;
- the two halves separate and stay in the scene as the physical result of the cut.

When all four objects are cut the workflow stops and a green message appears in the top-right corner. The application then waits: **Reset Auto Objects** restores the objects and the pose, and **Activate Tool** starts a new cutting cycle. **Start operation** and the manual pick/place buttons also stop the cutting workflow first.

### Radar

The arm does not keep a fixed list of objects. A built-in radar scans the pickup zone (`RadarScanner`) on every automated step and reports only the objects that are actually lying there - whole blocks as well as cut pieces. The scan is sorted by distance, and **Start operation** as well as the next pick always take the nearest detection. Each object brings its half extents, so the gripper approach height and the carried-payload envelope follow the real piece size.

The **RADAR - PICKUP ZONE** panel in the bottom-left corner shows what the radar currently sees: the object count and, per object, its size in centimetres and its `x`, `y`, `z` coordinates. When a piece is cut the count grows, and when an object is picked up it leaves the scan.

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
- `Services/RadarScanner.cs` - pickup-zone scan that feeds the automation with sizes and positions.
- `Services/CuttingController.cs` - laser cutting workflow (approach, cut, retract per object).
- `Services/CutPlanner.cs` - turns a cut object into two transportable pieces.
- `Services/TransportPathPlanner.cs` - layered grid route search for obstacle avoidance.
- `Services/KeyframePlayer.cs` - keyframe recording and Hermite trajectory playback.
- `Services/IDialogService.cs`, `Services/WpfDialogService.cs` - dialog abstraction and WPF implementation.
- `Mvvm/ObservableObject.cs`, `Mvvm/RelayCommand.cs` - `INotifyPropertyChanged` base and `ICommand`.
- `Rendering/RobotSceneRenderer.cs` - builds and updates the `Viewport3D` scene from view-model events.
- `Rendering/MeshBuilder.cs` - procedural mesh helpers.
- `Rendering/BoxSplitter.cs` - splits an object mesh along the cutting plane.
- `Views/CameraController.cs` - orbit and zoom camera.
- `Kinematics/ForwardKinematics.cs` - rotation, translation, and planar kinematics helpers.
- `Kinematics/InverseKinematics6D.cs` - damped-least-squares 6D IK solver.

## Build

The project targets `net10.0-windows` and uses nullable reference types and implicit usings. Build it from Visual Studio with **Build > Build Solution**.

Visual Studio, .NET build output, and local publish files are excluded through `.gitignore`.
