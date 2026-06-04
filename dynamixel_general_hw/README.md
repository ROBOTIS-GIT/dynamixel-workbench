# dynamixel_general_hw

`dynamixel_general_hw` is a `ros2_control` hardware plugin for Dynamixel actuators. It connects `dynamixel_workbench_toolbox` to `hardware_interface::SystemInterface`, so standard ROS 2 controllers such as `joint_trajectory_controller`, `velocity_controllers`, and `effort_controllers` can command Dynamixel joints.

## Environment

Build and run this package from the ROS 2 workspace.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws
colcon build --packages-select dynamixel_general_hw
source ~/colcon_ws/install/setup.bash
```

If you use a USB adapter, make sure the device is visible and writable.

```bash
ls -l /dev/ttyUSB0
```

## Samples

All samples accept following launch arguments.

- `port_name`: serial port connected to the Dynamixel bus. Default: `/dev/ttyUSB0`.
- `baud_rate`: Dynamixel baud rate. Default: `57600`.
- `protocol_1_0`: use Protocol 1.0 for samples that support it. Default: `false`.
- `dynamixel_id`: single-actuator ID for samples 1, 2, 4, 5, and 6.
- `pan_id`, `tilt_id`: actuator IDs for sample 3.

Protocol 1.0 samples skip `Operating_Mode` writes because older models do not have that control-table item. Set the actuator mode manually before launching when needed.


### Sample 1: simplest position control

This sample controls one Dynamixel actuator through `position_joint_trajectory_controller`. The sample joint name is `sample_joint`.

#### XC330-T181-T setup

This is the common Protocol 2.0 setup for the XC330-T181-T factory defaults listed by ROBOTIS e-Shop.

- Protocol: `2.0`
- ID: `1`
- Baud rate: `57600`
- Port: `/dev/ttyUSB0`
- The sample writes `Operating_Mode=3`, which is Position Control Mode.

```bash
ros2 launch dynamixel_general_hw sample1.launch.py \
  port_name:=/dev/ttyUSB0 baud_rate:=57600 dynamixel_id:=1
```

#### AX-12A setup

This setup has been verified with an AX-12A.

- Protocol: `1.0`
- ID: `0`
- Baud rate: `1000000`
- Port: `/dev/ttyUSB0`
- Actuator mode: Joint Mode. AX-12A does not have a Protocol 2.0
  `Operating_Mode` item, so set non-zero CW/CCW angle limits on the actuator.

```bash
ros2 launch dynamixel_general_hw sample1.launch.py \
  port_name:=/dev/ttyUSB0 baud_rate:=1000000 protocol_1_0:=true dynamixel_id:=0
```

The launch starts `ros2_control_node`, `robot_state_publisher`, RViz,
`joint_state_broadcaster`, and `position_joint_trajectory_controller`.

Send a position command through the FollowJointTrajectory action.

```bash
ros2 action send_goal \
  /position_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [sample_joint], points: [{positions: [0.78], velocities: [0.0], time_from_start: {sec: 5, nanosec: 0}}]}}"
```

The command above moves `sample_joint` to `0.78` rad in 5 seconds.

Check joint states with the following command.

```bash
ros2 topic echo /joint_states
```

The important configuration files are:

- `urdf/sample1.urdf`: robot model, `ros2_control` hardware plugin, Dynamixel ID, initial control-table writes, command interfaces, and state interfaces.
- `config/sample1_2/default_controllers.yaml`: controller manager and controller parameters.
- `launch/sample1.launch.py`: sample-specific wrapper around `launch/dynamixel_general_control.launch.py`.


### Sample 2: mechanical reduction and joint offset

Assumes one Dynamixel actuator with ID `1`, baud rate `57600`, connected to `/dev/ttyUSB0`.

```bash
ros2 launch dynamixel_general_hw sample2.launch.py port_name:=/dev/ttyUSB0 baud_rate:=57600 dynamixel_id:=1
```

For Protocol 1.0,

```bash
ros2 launch dynamixel_general_hw sample2.launch.py port_name:=/dev/ttyUSB0 baud_rate:=57600 protocol_1_0:=true dynamixel_id:=1
```

This sample uses the same controller setup as Sample 1, but `urdf/sample2.urdf` sets a transmission offset of `0.7853` rad and a mechanical reduction of `2.0`.
Compared with Sample 1, RViz shows the joint with an offset and commanded motion is scaled by the reduction.


### Sample 3: multiple actuators

Assumes two Dynamixel actuators with IDs `1` and `2`, baud rate `57600`, connected to `/dev/ttyUSB0`.

```bash
ros2 launch dynamixel_general_hw sample3.launch.py port_name:=/dev/ttyUSB0 baud_rate:=57600 pan_id:=1 tilt_id:=2
```

For Protocol 1.0, set both actuators to Joint Mode manually and launch:

```bash
ros2 launch dynamixel_general_hw sample3.launch.py port_name:=/dev/ttyUSB0 baud_rate:=57600 protocol_1_0:=true pan_id:=1 tilt_id:=2
```

Send a two-joint trajectory,

```bash
ros2 action send_goal \
  /position_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [sample_pan_joint, sample_tilt_joint], points: [{positions: [-0.78, 0.78], velocities: [0.0, 0.0], time_from_start: {sec: 10, nanosec: 0}}]}}"
```

### Sample 4: velocity control

Assumes one Dynamixel actuator with ID `1`, baud rate `57600`, connected to `/dev/ttyUSB0`.

```bash
ros2 launch dynamixel_general_hw sample4.launch.py port_name:=/dev/ttyUSB0 baud_rate:=57600 dynamixel_id:=0
```

For AX-12A and other Protocol 1.0 actuators, use Wheel Mode for continuous rotation by setting both CW Angle Limit and CCW Angle Limit to `0` on the actuator. Then launch with `protocol_1_0:=true`.

```bash
ros2 launch dynamixel_general_hw sample4.launch.py port_name:=/dev/ttyUSB0 baud_rate:=1000000 protocol_1_0:=true dynamixel_id:=0
```

Send a velocity command in rad/s as follows.

```bash
ros2 topic pub --once \
  /joint_group_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [1.0]}"
```

### Sample 5: effort control

Assumes one Dynamixel actuator with ID `1`, baud rate `57600`, connected to `/dev/ttyUSB0`. The actuator model must support current control through `Goal_Current`.

```bash
ros2 launch dynamixel_general_hw sample5.launch.py port_name:=/dev/ttyUSB0 baud_rate:=57600 dynamixel_id:=0
```

Send an effort command in Nm.

```bash
ros2 topic pub --once \
  /joint_group_effort_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.1]}"
```

The effort value is converted to current using `torque_constant` in the URDF. If the output torque must be accurate, set `torque_constant` for your actuator and mechanism instead of relying on the sample value.


### Sample 6: position control with effort limit

This sample corresponds to Current-based Position Control Mode on supported Dynamixel models. It assumes one actuator with ID `1`, baud rate `57600`, connected to `/dev/ttyUSB0`.

```bash
ros2 launch dynamixel_general_hw sample6.launch.py port_name:=/dev/ttyUSB0 baud_rate:=57600
```

Set the effort limit first as follows.

```bash
ros2 topic pub --once \
  /joint_group_effort_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.1]}"
```

Then send the position command.

```bash
ros2 action send_goal \
  /position_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [sample_joint], points: [{positions: [1.0], velocities: [0.0], time_from_start: {sec: 1, nanosec: 0}}]}}"
```

The actuator moves toward the commanded position while the current-derived effort command is limited by the value sent to the effort controller.

## Common launch file

`launch/dynamixel_general_control.launch.py` is the shared launch file used by all samples. It starts the ROS 2 control stack for a URDF/Xacro that contains a `ros2_control` block using this plugin.

Show launch arguments as follows.

```bash
ros2 launch dynamixel_general_hw dynamixel_general_control.launch.py --show-args
```

Arguments:

- `port_name`: Dynamixel serial port. Default: `/dev/ttyUSB0`.
- `baud_rate`: Dynamixel baud rate. Default: `57600`.
- `protocol_1_0`: passed to sample URDF/Xacro files. Default: `false`.
- `robot_description_file`: URDF/Xacro file to load.
- `controllers_file`: controller manager YAML file.
- `controllers_to_start`: space-separated controller names to spawn.
- `dynamixel_id`: single-actuator ID passed to compatible sample URDFs.
- `pan_id`, `tilt_id`: two actuator IDs passed to sample 3.
- `launch_rviz`: whether to start RViz. Default: `true`.
- `rvizconfig`: RViz config file.

For headless checks, disable RViz:

```bash
ros2 launch dynamixel_general_hw sample1.launch.py launch_rviz:=false
```

## Hardware Parameters

Use these parameters inside the `<hardware>` tag of the `ros2_control` block.

```xml
<hardware>
  <plugin>dynamixel_general_hw/DynamixelGeneralHw</plugin>
  <param name="port_name">/dev/ttyUSB0</param>
  <param name="baud_rate">57600</param>
  <param name="protocol_version">2.0</param>
  <param name="calculate_effort">true</param>
</hardware>
```

- `port_name` is required.
- `baud_rate` defaults to `57600` when omitted.
- `protocol_version` defaults to `2.0`; set `1.0` for Protocol 1.0 models.
- `calculate_effort` defaults to `true`; set `false` to leave effort state at the raw initialized value instead of calculating it from current/load.

Use these parameters inside each `<joint>` in the `ros2_control` block.

- `id`: required Dynamixel ID, from `0` to `255`.
- `torque_constant`: optional Nm/A factor used to convert current to effort and effort commands to current commands.
- Any other parameter name is treated as a Dynamixel control-table item and is written during configuration with torque off, for example `Operating_Mode` or `Return_Delay_Time`.

Supported command interfaces are `position`, `velocity`, and `effort`. Supported state interfaces are `position`, `velocity`, `effort`, `current`, `temperature` and `voltage`.

## Minimal URDF Example

```xml
<ros2_control name="DynamixelGeneralHw" type="system">
  <hardware>
    <plugin>dynamixel_general_hw/DynamixelGeneralHw</plugin>
    <param name="port_name">/dev/ttyUSB0</param>
    <param name="baud_rate">57600</param>
    <param name="calculate_effort">true</param>
  </hardware>

  <joint name="joint1">
    <param name="id">1</param>
    <param name="Return_Delay_Time">0</param>
    <param name="Operating_Mode">3</param>
    <param name="torque_constant">1.15</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <state_interface name="current"/>
    <state_interface name="temperature"/>
    <state_interface name="voltage"/>
  </joint>
</ros2_control>
```
