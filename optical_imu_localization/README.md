# HereFlow DroneCAN ROS 2 localizer

This ROS 2 package obtains **real sensor data directly from a HereFlow CAN
module**. It is a passive DroneCAN listener: it does not transmit commands or
change the flight controller or sensor configuration.

HereFlow broadcasts optical-flow angular displacement and lidar range. The node
uses the range measured at the same time to convert angular flow into metres,
then integrates the resulting displacement from the user-set local coordinate.
If HereFlow firmware is configured to publish `uavcan.equipment.ahrs.RawIMU`,
the node also publishes it and integrates its yaw gyro from the user-selected
initial yaw.

## Connections and data path

```text
HereFlow ── DroneCAN ── CAN transceiver / CAN adapter ── host (can0 or COM port)
                                                     │
                                                     └─ HereFlow localizer ── /odom, /pose, TF
```

The host must be physically connected to the same CAN bus as HereFlow (for
example through a correctly terminated CAN bus tap or a compatible CAN adapter).
Do not connect the module to a GPIO/serial port as if it were a raw UART device:
HereFlow uses DroneCAN over CAN.

## ROS interfaces

| Interface | Type | Purpose |
| --- | --- | --- |
| `CAN: com.hex.equipment.flow.Measurement` | DroneCAN | Real HereFlow optical flow and quality. |
| `CAN: uavcan.equipment.range_sensor.Measurement` | DroneCAN | Real HereFlow lidar range used to scale flow into metres. |
| `CAN: uavcan.equipment.ahrs.RawIMU` | DroneCAN | Real HereFlow IMU when its firmware publishes it. |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Initial/reset coordinate from the user or RViz. |
| `/odom` | `nav_msgs/msg/Odometry` | Local position estimate. |
| `/pose` | `geometry_msgs/msg/PoseStamped` | Current pose. |
| `/hereflow/imu/raw` | `sensor_msgs/msg/Imu` | Decoded raw IMU data, if available. |
| `/hereflow/body_velocity` | `geometry_msgs/msg/Vector3Stamped` | Flow-derived velocity in the HereFlow body frame. |
| `odom -> base_link` | TF | Current transform. |

## Install and run

Install the Python DroneCAN decoder on the computer that runs ROS 2:

```bash
python3 -m pip install dronecan
```

Place this package in your ROS 2 workspace `src` directory and build it:

```bash
colcon build --packages-select optical_imu_localization
source install/setup.bash
```

On Linux, configure the host CAN interface to the same bitrate as the HereFlow
bus, then start the node. `can0` is the usual SocketCAN name; PyDroneCAN also
supports compatible SLCAN adapters named by a serial device path.

```bash
ros2 launch optical_imu_localization localization.launch.py \
  can_interface:=can0 can_bitrate:=1000000 \
  initial_x:=2.5 initial_y:=-1.0 initial_z:=0.0 initial_yaw:=1.5708
```

Set `hereflow_node_id` to the module's DroneCAN node ID when more than one
DroneCAN sensor is present. The default `0` accepts all node IDs.

## Mounting, calibration, and limits

For the standard PX4/DroneCAN convention, forward module movement is represented
by positive `flow_integral[1]`, and leftward movement by positive
`flow_integral[0]`; the program applies this mapping before position integration.
Perform a short measured forward/left test before flight. Use `flow_scale` to
correct distance and negate `flow_forward_sign` or `flow_left_sign` only when
the measured directions are reversed.

The user-supplied `initial_yaw` is a local reference. RawIMU does not provide an
absolute compass heading, so yaw will drift over time if the module does not
provide a separate attitude/heading source. For robust long missions, fuse this
local odometry with a magnetometer, visual odometry, GPS, or a flight-controller
estimator. Some HereFlow firmware publishes only flow and range; in that case
the localizer holds the initial yaw and logs position only in that fixed frame.

## Live PC graph and repeatable simulation

The visualizer opens two graphs in one desktop window, both beginning at
`(0, 0, 0)`. The left graph shows the live X-Y travel path. The right graph
shows X and Y positions over time, so each incoming odometry change is visible
as it happens.

Install its desktop plotting dependency once:

```bash
python3 -m pip install matplotlib
```

Run it alongside the live HereFlow localizer:

```bash
ros2 launch optical_imu_localization visualization.launch.py odom_topic:=/odom
```

For a PC-only demonstration without CAN hardware, run:

```bash
ros2 launch optical_imu_localization demo_simulation.launch.py
```

The demonstration publishes `(0,0) → (1,0) → (2,0) → (2,1) → (2,2)` at
one-second intervals. Its path graph therefore visibly shows two steps along
the positive X axis (right) followed by two steps along the positive Y axis
(forward), exactly as requested. It publishes only `/simulated_odom`, never
changes or replaces the real `/odom` topic.
