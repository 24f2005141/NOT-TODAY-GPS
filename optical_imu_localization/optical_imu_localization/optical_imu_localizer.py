#!/usr/bin/env python3
"""Local odometry using a HereFlow module connected through DroneCAN.

HereFlow broadcasts its optical-flow, range, and (when enabled in firmware)
RawIMU data over DroneCAN. This node listens directly on a SocketCAN or SLCAN
interface using PyDroneCAN; it does not use simulated or hard-coded sensor
measurements. Optical flow is converted from line-of-sight radians to metres
using the contemporaneous range measurement.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import queue
import threading
import time
from pathlib import Path
from typing import Optional, Union

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


@dataclass(frozen=True)
class FlowSample:
    """One decoded HereFlow DroneCAN optical-flow transfer."""
    node_id: int
    integration_interval_s: float
    flow_x_rad: float
    flow_y_rad: float
    quality: int
    received_monotonic_s: float


@dataclass(frozen=True)
class RangeSample:
    """One decoded HereFlow DroneCAN range-sensor transfer."""
    node_id: int
    range_m: float
    received_monotonic_s: float


@dataclass(frozen=True)
class ImuSample:
    """One decoded DroneCAN RawIMU transfer from the HereFlow node."""
    node_id: int
    integration_interval_s: float
    gyro_x_rad_s: float
    gyro_y_rad_s: float
    gyro_z_rad_s: float
    gyro_z_integral_rad: float
    accel_x_mps2: float
    accel_y_mps2: float
    accel_z_mps2: float


@dataclass(frozen=True)
class CanError:
    """An error raised by the CAN receive thread, forwarded to ROS logging."""
    detail: str


CanEvent = Union[FlowSample, RangeSample, ImuSample, CanError]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return the planar yaw angle represented by a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    """Return z and w components for a zero-roll, zero-pitch quaternion."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class HereFlowLocalizer(Node):
    """Fuse real HereFlow DroneCAN flow, range, and IMU data into local odometry."""

    def __init__(self) -> None:
        super().__init__('hereflow_localizer')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 1_000_000)
        self.declare_parameter('hereflow_node_id', 0)
        self.declare_parameter('min_flow_quality', 100)
        self.declare_parameter('min_range_m', 0.08)
        self.declare_parameter('max_range_m', 3.0)
        self.declare_parameter('max_range_age_s', 0.25)
        self.declare_parameter('flow_scale', 1.0)
        self.declare_parameter('flow_forward_sign', 1.0)
        self.declare_parameter('flow_left_sign', 1.0)
        self.declare_parameter('imu_yaw_sign', 1.0)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.can_interface = str(self.get_parameter('can_interface').value)
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.hereflow_node_id = int(self.get_parameter('hereflow_node_id').value)
        self.min_flow_quality = int(self.get_parameter('min_flow_quality').value)
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.max_range_age_s = float(self.get_parameter('max_range_age_s').value)
        self.flow_scale = float(self.get_parameter('flow_scale').value)
        self.flow_forward_sign = float(self.get_parameter('flow_forward_sign').value)
        self.flow_left_sign = float(self.get_parameter('flow_left_sign').value)
        self.imu_yaw_sign = float(self.get_parameter('imu_yaw_sign').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        if self.can_bitrate <= 0 or self.min_range_m <= 0.0 or self.max_range_m <= self.min_range_m:
            raise ValueError('CAN bitrate and range limits must be positive, with max_range_m > min_range_m')

        self.latest_range: Optional[RangeSample] = None
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.have_hereflow_imu = False
        self._last_warning_s = 0.0
        self._events: queue.Queue[CanEvent] = queue.Queue()
        self._can_stop = threading.Event()
        self._can_thread: Optional[threading.Thread] = None
        self._dronecan_node = None

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 10)
        self.raw_imu_publisher = self.create_publisher(Imu, 'hereflow/imu/raw', 20)
        self.body_velocity_publisher = self.create_publisher(Vector3Stamped, 'hereflow/body_velocity', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initial_pose_callback, 10)

        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self.create_timer(0.01, self.drain_can_events)
        self.create_timer(1.0 / publish_rate, self.publish_state)
        self.start_can_receiver()
        self.get_logger().info(
            f'Listening for HereFlow DroneCAN data on {self.can_interface} at {self.can_bitrate} bit/s; '
            f'initial pose is x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, yaw={self.yaw:.3f} rad.'
        )

    def source_is_selected(self, node_id: int) -> bool:
        """Accept any HereFlow node for 0, otherwise only the configured node ID."""
        return self.hereflow_node_id == 0 or self.hereflow_node_id == node_id

    def start_can_receiver(self) -> None:
        """Start a passive PyDroneCAN listener in a dedicated receive thread."""
        try:
            import dronecan
        except ImportError as error:
            raise RuntimeError(
                'PyDroneCAN is required for direct HereFlow CAN input. Install it with: pip install dronecan'
            ) from error

        dsdl_directory = Path(get_package_share_directory('optical_imu_localization')) / 'dsdl' / 'com'
        if not dsdl_directory.is_dir():
            raise RuntimeError(f'HereFlow DSDL definitions are missing: {dsdl_directory}')
        dronecan.load_dsdl(str(dsdl_directory))

        try:
            flow_type = dronecan.com.hex.equipment.flow.Measurement
        except AttributeError as error:
            raise RuntimeError('Could not load the HereFlow com.hex.equipment.flow DroneCAN definition') from error

        # node_id=None selects passive mode: this program only observes HereFlow
        # transfers and never transmits commands or alters the CAN bus.
        self._dronecan_node = dronecan.make_node(
            self.can_interface, node_id=None, bitrate=self.can_bitrate
        )
        self._dronecan_node.add_handler(flow_type, self.can_flow_callback)
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.range_sensor.Measurement, self.can_range_callback
        )
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.ahrs.RawIMU, self.can_imu_callback
        )
        self._can_thread = threading.Thread(target=self.can_receive_loop, name='hereflow-can-rx', daemon=True)
        self._can_thread.start()

    def can_receive_loop(self) -> None:
        """Process DroneCAN transfers without blocking the ROS executor."""
        try:
            import dronecan
            while not self._can_stop.is_set():
                try:
                    self._dronecan_node.spin(0.1)
                except dronecan.transport.TransferError:
                    # A corrupt CAN transfer must not stop valid subsequent data.
                    continue
        except Exception as error:  # Error is reported by the ROS-thread timer.
            self._events.put(CanError(str(error)))

    def can_flow_callback(self, event) -> None:
        """Queue a real HereFlow optical-flow transfer decoded by PyDroneCAN."""
        if not self.source_is_selected(event.transfer.source_node_id):
            return
        message = event.message
        self._events.put(FlowSample(
            node_id=event.transfer.source_node_id,
            integration_interval_s=float(message.integration_interval),
            flow_x_rad=float(message.flow_integral[0]),
            flow_y_rad=float(message.flow_integral[1]),
            quality=int(message.quality),
            received_monotonic_s=time.monotonic(),
        ))

    def can_range_callback(self, event) -> None:
        """Queue a valid real HereFlow lidar range transfer."""
        if not self.source_is_selected(event.transfer.source_node_id):
            return
        message = event.message
        valid_reading = getattr(message, 'READING_TYPE_VALID_RANGE', 1)
        if int(message.reading_type) != int(valid_reading):
            return
        self._events.put(RangeSample(
            node_id=event.transfer.source_node_id,
            range_m=float(message.range),
            received_monotonic_s=time.monotonic(),
        ))

    def can_imu_callback(self, event) -> None:
        """Queue a real HereFlow RawIMU transfer, if enabled by its firmware."""
        if not self.source_is_selected(event.transfer.source_node_id):
            return
        message = event.message
        self._events.put(ImuSample(
            node_id=event.transfer.source_node_id,
            integration_interval_s=float(message.integration_interval),
            gyro_x_rad_s=float(message.rate_gyro_latest[0]),
            gyro_y_rad_s=float(message.rate_gyro_latest[1]),
            gyro_z_rad_s=float(message.rate_gyro_latest[2]),
            gyro_z_integral_rad=float(message.rate_gyro_integral[2]),
            accel_x_mps2=float(message.accelerometer_latest[0]),
            accel_y_mps2=float(message.accelerometer_latest[1]),
            accel_z_mps2=float(message.accelerometer_latest[2]),
        ))

    def drain_can_events(self) -> None:
        """Apply receive-thread events safely in the ROS executor thread."""
        while True:
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                return
            if isinstance(event, FlowSample):
                self.apply_flow(event)
            elif isinstance(event, RangeSample):
                if self.min_range_m <= event.range_m <= self.max_range_m:
                    self.latest_range = event
            elif isinstance(event, ImuSample):
                self.apply_imu(event)
            else:
                self.get_logger().error(f'DroneCAN receive failure: {event.detail}')

    def warn_throttled(self, detail: str) -> None:
        """Avoid flooding the console when a sensor temporarily becomes invalid."""
        now = time.monotonic()
        if now - self._last_warning_s >= 2.0:
            self.get_logger().warn(detail)
            self._last_warning_s = now

    def apply_flow(self, sample: FlowSample) -> None:
        """Convert HereFlow angular flow and lidar height into local displacement."""
        if sample.quality < self.min_flow_quality:
            self.warn_throttled(f'Ignoring HereFlow flow quality {sample.quality} below {self.min_flow_quality}')
            return
        if sample.integration_interval_s <= 0.0:
            self.warn_throttled('Ignoring HereFlow flow transfer with a non-positive integration interval')
            return
        if self.latest_range is None or self.latest_range.node_id != sample.node_id:
            self.warn_throttled('Ignoring HereFlow flow: no valid range measurement from the same CAN node')
            return
        if sample.received_monotonic_s - self.latest_range.received_monotonic_s > self.max_range_age_s:
            self.warn_throttled('Ignoring HereFlow flow: latest range measurement is stale')
            return

        # DroneCAN/PX4 convention: forward motion produces +flow_integral[1],
        # and leftward motion produces +flow_integral[0]. Multiply angular LOS
        # motion by range to obtain metres in the module body frame.
        body_forward_m = sample.flow_y_rad * self.latest_range.range_m * self.flow_scale * self.flow_forward_sign
        body_left_m = sample.flow_x_rad * self.latest_range.range_m * self.flow_scale * self.flow_left_sign
        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        dx = cos_yaw * body_forward_m - sin_yaw * body_left_m
        dy = sin_yaw * body_forward_m + cos_yaw * body_left_m
        self.x += dx
        self.y += dy
        self.last_vx = dx / sample.integration_interval_s
        self.last_vy = dy / sample.integration_interval_s

        velocity = Vector3Stamped()
        velocity.header.stamp = self.get_clock().now().to_msg()
        velocity.header.frame_id = self.base_frame
        velocity.vector.x = body_forward_m / sample.integration_interval_s
        velocity.vector.y = body_left_m / sample.integration_interval_s
        self.body_velocity_publisher.publish(velocity)

    def apply_imu(self, sample: ImuSample) -> None:
        """Publish HereFlow RawIMU and integrate its yaw gyro from the user yaw."""
        if sample.integration_interval_s > 0.0:
            yaw_delta = sample.gyro_z_integral_rad
        else:
            # RawIMU permits no integrated samples; fall back to the latest rate.
            yaw_delta = sample.gyro_z_rad_s * 0.01
        self.yaw = normalize_angle(self.yaw + self.imu_yaw_sign * yaw_delta)
        self.have_hereflow_imu = True

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.base_frame
        imu.orientation_covariance[0] = -1.0  # RawIMU has no absolute orientation.
        imu.angular_velocity.x = sample.gyro_x_rad_s
        imu.angular_velocity.y = sample.gyro_y_rad_s
        imu.angular_velocity.z = sample.gyro_z_rad_s
        imu.linear_acceleration.x = sample.accel_x_mps2
        imu.linear_acceleration.y = sample.accel_y_mps2
        imu.linear_acceleration.z = sample.accel_z_mps2
        self.raw_imu_publisher.publish(imu)

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Reset the estimate to a coordinate supplied by the operator or RViz."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.get_logger().info(
            f'Initial pose reset to x={self.x:.3f}, y={self.y:.3f}, z={self.z:.3f}, yaw={self.yaw:.3f} rad'
        )

    def publish_state(self) -> None:
        """Publish pose, odometry, and odom-to-base TF from the current estimate."""
        now = self.get_clock().now().to_msg()
        qz, qw = quaternion_from_yaw(self.yaw)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.last_vx
        odom.twist.twist.linear.y = self.last_vy
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[14] = 1.0
        odom.pose.covariance[35] = 0.5 if self.have_hereflow_imu else 2.0
        self.odom_publisher.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_publisher.publish(pose)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def destroy_node(self) -> bool:
        """Stop the passive CAN receiver before releasing ROS resources."""
        self._can_stop.set()
        if self._can_thread is not None:
            self._can_thread.join(timeout=1.0)
        if self._dronecan_node is not None:
            self._dronecan_node.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = HereFlowLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
