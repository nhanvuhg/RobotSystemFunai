# AI Coding Agent Instructions — ROS2 Robot Control (nova5)

Purpose: concise, actionable guidance for AI agents working on the `robot_control_main` package.
- Build (workspace root `/home/pi/ros2_ws`):
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_control_main
source install/setup.bash
```

- Run main node:

```bash
ros2 run robot_control_main robot_logic_node
```

Architecture (big picture)
- Cameras → YOLO (Hailo) → `Detection2DArray` → `robot_logic_node` → Dobot via `dobot_msgs_v3` services.
- Main components to inspect: `src/robot_logic_node.cpp`, YOLO package (`yolo_ros_hailort_cpp`), `config/joint_pose_params.yaml`.

Key code patterns & conventions (concrete)
- State machine: `SystemState` enum in `src/robot_logic_node.cpp` controls high-level flow (INIT, WAIT_FILLING, TAKE_CHAMBER_TO_SCALE, PLACE_TO_OUTPUT, ERROR_*).
- Threading: state machine runs on a dedicated `std::thread` with `std::atomic<bool> state_machine_running_`.
- ROI & filtering: `ROIQuad` (polygon + bbox) and `RowFilter` (median window, `window=5`, `max_fall=2`, `ready_consec=3`) — use these for spatial filtering and debounced row readiness.
- Service clients use a consistent namespace: `/nova5/dobot_bringup/<ServiceName>` (e.g. `JointMovJ`, `RelMovL`, `DO`).
- Motion parameters are YAML-driven; load via `loadMotionParameters()` into `joint_sequences_` and `relmovl_sequences_`.
- Simulation flags: `simulate_scale_` and `force_pass_` change scale behavior for testing.

Integration & debugging tips (project-specific)
- Watch detection messages: `ros2 topic echo /camera1/detections` to verify `Detection2DArray` content.
- Call robot services manually for reproduction (example):

```bash
ros2 service call /nova5/dobot_bringup/EnableRobot dobot_msgs_v3/srv/EnableRobot "{enable: true}"
```

- Use `ros2 topic echo /error` and node logs (`RCLCPP_*`) to trace state transitions.
- For camera switching use provided scripts `cam0.sh` / `cam1.sh` or the `/switch_camera` service if present.

Files to inspect (quick links)
- `src/robot_logic_node.cpp` — main state machine, ROI and filtering examples.
- `config/joint_pose_params.yaml` — canonical motion definitions.
- `yolov8s.hef` & `input_1_yolov8s.hef` — Hailo model files (model loading/paths found in YOLO package).

What to change or verify when editing
- Keep state transitions explicit and guarded by atomic flags; avoid long blocking calls inside the state thread without timeouts.
- When modifying ROI logic, preserve `bbox_contains` fast-reject before polygon tests.
- If adding new services/topics, mirror existing namespace conventions (`/nova5/...`) and add corresponding parameterized names.

When you finish a change
- Build with `colcon build --packages-select robot_control_main` and run `ros2 run robot_control_main robot_logic_node`.
- Validate with `ros2 topic echo` and `ros2 service call` as above.

If anything below is unclear or you want an expanded section (examples, unit-test harness, or a checklist for PRs), tell me which part to expand.

# Safety rules for automated edits (DO NOT MODIFY)

- **Do not clear `selected_input_row_` or `selected_output_slot_` in `gotoStateCallback` or other pre-transition hooks.**
	- State handlers consume these selections; clearing them in the callback races with the handler and leads to wrong pick indices.

- **Preserve YAML index mapping for rows.**
	- `motion_sequence` index 0 = HOME. Input tray rows are at indices **1..5**. When picking Row N (1..5) call `moveToIndex(N)` (do NOT subtract 1).

- **Do not change mutex/atomic patterns around state machine without review.**
	- `state_machine_running_`, `row_selection_mutex_`, and `output_slot_selection_mutex_` coordinate cross-thread actions — altering them can introduce race conditions.

- **If you reorder or change `motion_sequence` entries, update in-code comments and mapping.**
	- Any change to ordering must be reflected where indices are used (search for `moveToIndex(...)` and the comment "INDEX 0 is HOME").

- **When adding logs or debug prints, keep them non-blocking and minimal.**
	- Avoid heavy blocking inside the `stateMachineLoop` or service callbacks.

- **Validation before modifying motion logic:**
	- Build and run the node; use these commands to validate behavior after edits:
		```bash
		colcon build --packages-select robot_control_main
		source install/setup.bash
		ros2 run robot_control_main robot_logic_node --ros-args --params-file src/robot_control_main/config/joint_pose_params.yaml
		```
	- Use `ros2 topic pub` + `ros2 topic echo` to reproduce manual commands and verify `moveToIndex` logs.

- **If unsure, ask the repository owner before large changes.**

# AI Coding Agent Instructions for ROS2 Robot Control System

## Architecture Overview
This is a ROS2-based pill dispensing robot system using Dobot Magician with dual CSI cameras and Hailo AI accelerator for YOLO object detection.

**Key Components:**
- `robot_control_main`: Core logic node implementing state machine for dispensing workflow
- `yolo_ros_hailort_cpp`: YOLOv8 inference on HailoRT for pill detection
- `csi_camera`: CSI camera driver for Raspberry Pi
- `ros2_qml_gui1`: QML-based user interface
- `bbox_drawer_cpp`: Visualization overlay for detections

**Data Flow:**
CSI Cameras → YOLO Detection Nodes → Detection2DArray messages → Robot Logic Node → Dobot Service Calls

## State Machine Workflow
The system follows a strict state machine in `robot_logic_node.cpp`:
- INIT sequence: Check/load chambers, refill buffer
- MAIN LOOP: Wait for filling → Take to scale → Wait result → Place to output/fail
- BUFFER MANAGEMENT: Refill/load as needed
- ERROR HANDLING: Scale errors, timeouts

States are defined in `SystemState` enum; transitions based on sensor inputs and detection counts.

## Motion Control
**Parameter Format** (in `config/joint_pose_params.yaml`):
- `J,j1,j2,j3,j4,j5,j6`: Joint angles for Dobot
- `C,x,y,z,rx,ry,rz`: Cartesian coordinates
- `R,dx,dy,dz`: Relative moves
- `D,index,status`: Digital outputs (gripper)

All motion sequences indexed; robot uses `JointMovJ`, `RelMovL`, `DO` services from `dobot_msgs_v3`.

## Vision Integration
- Subscribes to `Detection2DArray` from `/camera1/detections` and `/camera2/detections`
- Uses ROI structs (`ROIQuad`) for spatial filtering of detections
- Row-based filtering with `RowFilter` struct (median + hysteresis)
- OpenCV for coordinate transforms and ROI checks

## Build & Run Workflow
```bash
# Build (from /home/pi/ros2_ws)
colcon build --packages-select robot_control_main

# Source environments
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch full system
ros2 launch yolo_ros_hailort_cpp system_csi_dual_model.launch.py

# Run individual nodes
ros2 run robot_control_main robot_logic_node
```

## Camera Management
- Switch cameras: `ros2 service call /switch_camera std_srvs/srv/SetBool "{data: false}"`
- Hardware switch scripts: `~/cam0.sh`, `~/cam1.sh` (i2c commands)
- Models: `yolov8s.hef`, `input_1_yolov8s.hef` for Hailo inference

## Dependencies & Conventions
- **ROS2 Jazzy** with `dobot_msgs_v3`, `vision_msgs`, `cv_bridge`
- **OpenCV** for image processing (ROI calculations)
- **Threading**: State machine runs in separate thread with `std::atomic<bool>` control
- **Error Handling**: Publishes to `/error` topic; emergency stop service available
- **Configuration**: All motion params in YAML; load via `loadMotionParameters()`

## Key Files
- `src/robot_logic_node.cpp`: Main state machine and robot control logic
- `config/joint_pose_params.yaml`: All robot poses and motion sequences
- `scripts/enable_robot.sh`: Robot power control
- `nova5.launch.py`: System launch configuration (currently empty)

## Development Notes
- Robot namespace: `"nova5_HP"` (configurable)
- First batch handling: Special init sequence for startup
- Manual mode: Service to override automatic control
- Scale integration: Boolean result subscription for weight validation</content>
<parameter name="filePath">/home/pi/ros2_ws/.github/copilot-instructions.md