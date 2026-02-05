## ROS Packages

Baru Simulasi Gazebo, langsung run launch file buat cek

### How to Run
Run the launch file
```
ros2 launch turret_simulation gazebo_launch.py
```

### How to Control
Try Publish joint trajectory on another terminal
```
ros2 topic pub /turret_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{ 
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  joint_names: ['Horizontal_Joint', 'Vertical_Joint'],
  points: [
    {
      positions: [-0.5, -0.2],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}" --once --qos-reliability best_effort

```

Change controller
```
ros2 control ros2 control switch_controllers --deactivate turret_controller --activate turret_velocity_controller 
```

Try publish velocity
```
ros2 topic pub /turret_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [-1.0, 0.0]}"
```