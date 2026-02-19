# Steeldome Firmware
ESP32 firmware for the Steeldome turret: stepper control, AS5600 encoders, servo, and micro-ROS integration.
## Features
- micro-ROS node for telemetry and commands (`/motor/*`, `/stepper/pos`, `/servo_cmd`, `/led_cmd`, `/reset_pos`)
- Stepper control via `setupStepper()` / `runStepper()` ([src/stepper.cpp](src/stepper.cpp))
- AS5600 encoder handling via `setupEncoder()`, `updateEncoderPosition()`, `saveEncoderPosition()`, `loadEncoderPosition()` ([src/encoder.cpp](src/encoder.cpp))
- Persistent position storage using `Preferences` (see [src/config.h](src/config.h))
- PID and motor driver helpers in `lib/` (`lib/PID_Controller`, `lib/L298N_Driver`)
## Quick Start
Prerequisites:
- PlatformIO (PIO CLI) installed
- USB connection to the board (usually `/dev/ttyUSB0` on Linux)

Build & Upload:
- Build with PlatformIO: use Ctrl+Alt+B (or `platformio run`)
- Upload/Flash: use Ctrl+Alt+U (or `platformio run -t upload -e esp32doit-devkit-v1`)

micro-ROS agent:
Run the micro-ROS agent on your host with:

```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
```

For a full micro-ROS tutorial and agent setup see: https://micro-ros.github.io/docs/tutorials/core/first_application_linux/

## Topics, Services and Examples

- Publishers from the ESP32:
	- `/stepper/pos` — `std_msgs/msg/Float64MultiArray` (stepper positions)
	- `/motor/vel` — `std_msgs/msg/Float64MultiArray` (motor velocities)
	- `/motor/pos` — `std_msgs/msg/Float64MultiArray` (motor positions)
- Subscriptions to the ESP32:
	- `/motor/cmd_vel` — `std_msgs/msg/Float64MultiArray` (command velocities)
	- `/motor/cmd_pos` — `std_msgs/msg/Float64MultiArray` (command positions)
	- `/servo_cmd` — `std_msgs/msg/Int32` (servo angle)
	- `/led_cmd` — `std_msgs/msg/String` (LED text commands)
- Service:
	- `/reset_pos` — `std_srvs/srv/Empty`

Example publishes (from the host with ROS 2 and agent running):

```sh
# Publish pitch and yaw target with turret_kinematics_node running
ros2 topic pub /turret/cmd_pos turret_msgs/msg/TurretState "{pitch: 0.0, yaw: 0.0, fire_command: false}"

# Publish two-element Float64MultiArray to set motor positions
ros2 topic pub /motor/cmd_pos std_msgs/msg/Float64MultiArray "{data: [0.0, 90.0]}" -1

# Move servo to 45
ros2 topic pub /servo_cmd std_msgs/msg/Int32 "{data: 45}" -1

# Call reset service
ros2 service call /reset_pos std_srvs/srv/Empty "{}"
```
Adjust message syntax if your ROS 2 CLI/format differs; the above is a typical YAML-style inline example.

## Configuration & Important Constants

- `platformio.ini` contains the upload and micro-ROS transport configuration (see `board_microros_transport`, `board_microros_distro`).
- Persistent save interval: `SAVE_INTERVAL_MS` in [src/config.h](src/config.h).
- Main RTOS tasks: main loop runs stepper/encoder on one core, ROS runs on a pinned core (`rosTask` in [src/main.cpp](src/main.cpp)).

## Key Files
- [platformio.ini](platformio.ini)
- [src/main.cpp](src/main.cpp) — entry point; contains `rosTask` and main loop
- [src/ros_node.cpp](src/ros_node.cpp) — micro-ROS setup and callbacks
- [src/encoder.cpp](src/encoder.cpp) — AS5600 handling and persistence
- [src/stepper.cpp](src/stepper.cpp) — AccelStepper control
- [src/config.h](src/config.h) — constants and shared state

## Contributing
If you want to add a new micro-ROS publisher, subscriber, or service, follow these guidelines:

- **Where to edit:** Start from [src/ros_node.cpp](src/ros_node.cpp) and [src/ros_node.h](src/ros_node.h).
- **Publisher:** Create an `rcl_publisher_t` (or use rclc helpers) during node initialization and publish from a suitable task or timer.
- **Subscriber:** Create an `rcl_subscription_t` and register a callback that safely updates shared state (use FreeRTOS primitives if needed).
- **Service:** Create an `rcl_service_t` and implement a request handler registered during initialization.
- **Testing:** Build with `Ctrl+Alt+B` (or `platformio run`) and run the agent on your host with:
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baudrate 115200
```

Then use ROS 2 tools on the host to confirm topics and services (for example, `ros2 topic list`, `ros2 topic echo`, `ros2 service call`).

---
Last updated: 2026-02-19
