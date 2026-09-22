"""ROS 2 node for smoothed bimanual IK, fingers, and safe Gazebo velocity."""

from dataclasses import dataclass
import math
import os
import time
import xml.etree.ElementTree as ET
from typing import Dict, Optional, Sequence

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .filters import PoseEMA, ScalarEMA
from .kinematics import IKResult, SerialChain
from .math3d import pose_matrix

@dataclass
class PoseTarget:
    message: PoseStamped
    arrival: float

@dataclass
class ArmRuntime:
    side: str
    chain: SerialChain
    filter: PoseEMA
    solution: np.ndarray
    command: Optional[np.ndarray] = None
    was_stale: bool = True

class ETParseError(ValueError):
    pass

def _optional_names(values: Sequence[str], sentinel: str):
    names = tuple(str(value) for value in values)
    if not names or names == (sentinel,):
        return None
    return names

def _clamp(value, magnitude):
    return max(-magnitude, min(magnitude, float(value)))

def _duration(seconds):
    nanoseconds = int(round(seconds * 1_000_000_000))
    return Duration(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)

class R1KinematicsControl(Node):
    def __init__(self):
        super().__init__('r1_kinematics_control')
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._left_target = None
        self._right_target = None
        self._arms = {}
        self._joint_positions = {}
        self._description_hash = None
        self._teleop_active = False
        self._active_arrival = None
        self._velocity_arrival = None
        self._velocity_streaming = False
        self._joy_arrival = None
        self._left_closure_target = 0.0
        self._right_closure_target = 0.0
        self._left_closure_filter = ScalarEMA()
        self._right_closure_filter = ScalarEMA()
        self._last_tick = time.monotonic()
        self._warning_times = {}

        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        description_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self._arm_publisher = self.create_publisher(JointTrajectory, self._arm_command_topic, command_qos)
        self._left_hand_publisher = self.create_publisher(JointTrajectory, self._left_hand_command_topic, command_qos)
        self._right_hand_publisher = self.create_publisher(JointTrajectory, self._right_hand_command_topic, command_qos)
        self._cmd_vel_publisher = self.create_publisher(Twist, self._cmd_vel_output_topic, command_qos)

        self.create_subscription(PoseStamped, self._left_pose_topic, self._left_pose_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self._right_pose_topic, self._right_pose_callback, qos_profile_sensor_data)
        self.create_subscription(Joy, self._joy_topic, self._joy_callback, qos_profile_sensor_data)
        self.create_subscription(JointState, self._joint_states_topic, self._joint_state_callback, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, self._cmd_vel_input_topic, self._velocity_callback, command_qos)
        self.create_subscription(Bool, self._active_topic, self._active_callback, command_qos)
        self.create_subscription(String, self._robot_description_topic, self._description_callback, description_qos)

        self._control_timer = self.create_timer(1.0 / self._control_rate_hz, self._control_tick)

        self._load_initial_description()
        self.get_logger().info('R1 kinematics control started in safe state; waiting for deadman')

    def _declare_parameters(self):
        self.declare_parameter('max_ik_position_error_m', 0.08)
        self.declare_parameter('ik.max_iterations', 30)
        self.declare_parameter('ik.damping', 0.03)
        self.declare_parameter('ik.singularity_threshold', 0.05)
        self.declare_parameter('ik.max_joint_step_rad', 0.15)
        self.declare_parameter('ik.position_tolerance_m', 0.005)
        self.declare_parameter('ik.orientation_tolerance_rad', 0.04)
        self.declare_parameter('ik.orientation_weight', 0.35)

        self.declare_parameter('robot_description', '')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('left_arm.base_link', 'torso_link')
        self.declare_parameter('left_arm.tip_link', 'left_hand_link')
        self.declare_parameter('left_arm.joint_names', ['AUTO'])
        self.declare_parameter('right_arm.base_link', 'torso_link')
        self.declare_parameter('right_arm.tip_link', 'right_hand_link')
        self.declare_parameter('right_arm.joint_names', ['AUTO'])

        self.declare_parameter('left_hand.joint_names', ['DISABLED'])
        self.declare_parameter('left_hand.open_positions', [0.0])
        self.declare_parameter('left_hand.closed_positions', [1.0])
        self.declare_parameter('right_hand.joint_names', ['DISABLED'])
        self.declare_parameter('right_hand.open_positions', [0.0])
        self.declare_parameter('right_hand.closed_positions', [1.0])
        self.declare_parameter('left_trigger_axis_index', -1)
        self.declare_parameter('right_trigger_axis_index', -1)
        self.declare_parameter('left_trigger_button_index', 1)
        self.declare_parameter('right_trigger_button_index', 2)
        self.declare_parameter('trigger_axis_released_value', 0.0)
        self.declare_parameter('trigger_axis_pressed_value', 1.0)

        self.declare_parameter('left_pose_topic', '/vr/left_controller/pose')
        self.declare_parameter('right_pose_topic', '/vr/right_controller/pose')
        self.declare_parameter('joy_topic', '/vr/joy')
        self.declare_parameter('active_topic', '/vr/teleop/active')
        self.declare_parameter('cmd_vel_input_topic', '/vr/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('arm_command_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('left_hand_command_topic', '/left_hand_controller/joint_trajectory')
        self.declare_parameter('right_hand_command_topic', '/right_hand_controller/joint_trajectory')
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)
        self.declare_parameter('control_rate_hz', 60.0)
        self.declare_parameter('pose_timeout_sec', 0.25)
        self.declare_parameter('joy_timeout_sec', 0.50)
        self.declare_parameter('active_timeout_sec', 1.50)
        self.declare_parameter('cmd_vel_timeout_sec', 0.25)
        self.declare_parameter('position_ema_tau_sec', 0.075)
        self.declare_parameter('orientation_ema_tau_sec', 0.10)
        self.declare_parameter('finger_ema_tau_sec', 0.12)
        self.declare_parameter('trajectory_time_sec', 0.10)
        self.declare_parameter('max_joint_velocity_rad_s', 1.0)
        self.declare_parameter('workspace_reach_scale', 0.98)

    def _read_parameters(self):
        value = lambda name: self.get_parameter(name).value
        self._control_rate_hz = float(value('control_rate_hz'))
        self._pose_timeout_sec = float(value('pose_timeout_sec'))
        self._joy_timeout_sec = float(value('joy_timeout_sec'))
        self._active_timeout_sec = float(value('active_timeout_sec'))
        self._cmd_vel_timeout_sec = float(value('cmd_vel_timeout_sec'))
        self._position_tau = float(value('position_ema_tau_sec'))
        self._orientation_tau = float(value('orientation_ema_tau_sec'))
        self._finger_tau = float(value('finger_ema_tau_sec'))
        self._trajectory_time_sec = float(value('trajectory_time_sec'))
        self._max_joint_velocity = float(value('max_joint_velocity_rad_s'))
        self._workspace_reach_scale = float(value('workspace_reach_scale'))
        self._max_ik_position_error = float(value('max_ik_position_error_m'))
        self._ik_options = {
            'max_iterations': int(value('ik.max_iterations')),
            'damping': float(value('ik.damping')),
            'singularity_threshold': float(value('ik.singularity_threshold')),
            'max_joint_step': float(value('ik.max_joint_step_rad')),
            'position_tolerance': float(value('ik.position_tolerance_m')),
            'orientation_tolerance': float(value('ik.orientation_tolerance_rad')),
            'orientation_weight': float(value('ik.orientation_weight')),
        }
        self._robot_description = str(value('robot_description'))
        self._urdf_path = str(value('urdf_path'))
        self._robot_description_topic = str(value('robot_description_topic'))
        self._arm_config = {
            'left': {
                'base': str(value('left_arm.base_link')),
                'tip': str(value('left_arm.tip_link')),
                'joints': _optional_names(value('left_arm.joint_names'), 'AUTO'),
            },
            'right': {
                'base': str(value('right_arm.base_link')),
                'tip': str(value('right_arm.tip_link')),
                'joints': _optional_names(value('right_arm.joint_names'), 'AUTO'),
            },
        }
        self._left_hand = self._hand_config('left')
        self._right_hand = self._hand_config('right')
        self._left_trigger_axis = int(value('left_trigger_axis_index'))
        self._right_trigger_axis = int(value('right_trigger_axis_index'))
        self._left_trigger_button = int(value('left_trigger_button_index'))
        self._right_trigger_button = int(value('right_trigger_button_index'))
        self._trigger_released = float(value('trigger_axis_released_value'))
        self._trigger_pressed = float(value('trigger_axis_pressed_value'))
        self._left_pose_topic = str(value('left_pose_topic'))
        self._right_pose_topic = str(value('right_pose_topic'))
        self._joy_topic = str(value('joy_topic'))
        self._active_topic = str(value('active_topic'))
        self._cmd_vel_input_topic = str(value('cmd_vel_input_topic'))
        self._cmd_vel_output_topic = str(value('cmd_vel_output_topic'))
        self._joint_states_topic = str(value('joint_states_topic'))
        self._arm_command_topic = str(value('arm_command_topic'))
        self._left_hand_command_topic = str(value('left_hand_command_topic'))
        self._right_hand_command_topic = str(value('right_hand_command_topic'))
        self._max_forward = float(value('max_forward_mps'))
        self._max_lateral = float(value('max_lateral_mps'))
        self._max_yaw = float(value('max_yaw_rps'))

    def _hand_config(self, side):
        value = lambda suffix: self.get_parameter(f'{side}_hand.{suffix}').value
        names = _optional_names(value('joint_names'), 'DISABLED')
        opened = np.asarray(value('open_positions'), dtype=float)
        closed = np.asarray(value('closed_positions'), dtype=float)
        return {'names': names, 'open': opened, 'closed': closed}

    def _validate_parameters(self):
        if self._control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        if not 0.0 < self._workspace_reach_scale <= 1.0:
            raise ValueError('workspace_reach_scale must be in (0, 1]')

    def _load_initial_description(self):
        if self._robot_description.strip():
            self._configure_chains(self._robot_description)
            return
        if not self._urdf_path:
            self.get_logger().warning(f'No URDF loaded; waiting on {self._robot_description_topic}')
            return
        path = os.path.abspath(os.path.expanduser(self._urdf_path))
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                self._configure_chains(stream.read())
        except OSError as exc:
            self.get_logger().error(f'Cannot read URDF {path!r}: {exc}')

    def _description_callback(self, message):
        if message.data.strip():
            self._configure_chains(message.data)

    def _configure_chains(self, description):
        description_hash = hash(description)
        if description_hash == self._description_hash:
            return
        try:
            configured = {}
            for side in ('left', 'right'):
                config = self._arm_config[side]
                chain = SerialChain.from_urdf(description, config['base'], config['tip'], config['joints'])
                seed = self._seed_for_chain(chain)
                configured[side] = ArmRuntime(side=side, chain=chain, filter=PoseEMA(), solution=seed)
        except (ValueError, ET.ParseError) as exc:
            self.get_logger().error(f'URDF chain configuration rejected: {exc}')
            return

        self._arms = configured
        self._description_hash = description_hash
        for side, runtime in self._arms.items():
            self.get_logger().info(f'{side} IK chain configured with reach={runtime.chain.nominal_reach:.3f} m')

    def _seed_for_chain(self, chain):
        seed = chain.neutral_positions()
        for index, name in enumerate(chain.joint_names):
            if name in self._joint_positions:
                seed[index] = self._joint_positions[name]
        return np.clip(seed, chain.lower, chain.upper)

    def _left_pose_callback(self, message):
        self._left_target = PoseTarget(message=message, arrival=time.monotonic())

    def _right_pose_callback(self, message):
        self._right_target = PoseTarget(message=message, arrival=time.monotonic())

    def _joint_state_callback(self, message):
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                self._joint_positions[name] = float(position)

    def _active_callback(self, message):
        self._active_arrival = time.monotonic()
        self._teleop_active = bool(message.data)
        if not self._teleop_active:
            self._publish_zero_velocity()
            
    def _velocity_callback(self, message):
        self._velocity_arrival = time.monotonic()
        if not self._is_enabled(time.monotonic()):
            return
        twist = Twist()
        twist.linear.x = _clamp(message.twist.linear.x, self._max_forward)
        twist.linear.y = _clamp(message.twist.linear.y, self._max_lateral)
        twist.angular.z = _clamp(message.twist.angular.z, self._max_yaw)
        self._cmd_vel_publisher.publish(twist)

    def _joy_callback(self, message):
        self._joy_arrival = time.monotonic()
        self._left_closure_target = self._trigger_value(message, self._left_trigger_axis, self._left_trigger_button, 'left')
        self._right_closure_target = self._trigger_value(message, self._right_trigger_axis, self._right_trigger_button, 'right')

    def _trigger_value(self, message, axis_index, button_index, side):
        if axis_index >= 0 and axis_index < len(message.axes):
            raw = float(message.axes[axis_index])
            normalized = ((raw - self._trigger_released) / (self._trigger_pressed - self._trigger_released))
            return max(0.0, min(1.0, normalized))
        if button_index >= 0 and button_index < len(message.buttons):
            return 1.0 if message.buttons[button_index] else 0.0
        return 0.0

    def _control_tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        if not self._is_enabled(now):
            if self._velocity_streaming:
                self._publish_zero_velocity()
            return

        self._velocity_streaming = True
        self._publish_hand_trajectories(now, dt)

        arm_names = []
        arm_positions = []
        for side, runtime in self._arms.items():
            target = self._left_target if side == 'left' else self._right_target
            command = self._solve_arm(runtime, target, now, dt)
            if command is not None:
                arm_names.extend(runtime.chain.joint_names)
                arm_positions.extend([float(x) for x in command])
        
        if arm_names:
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = arm_names
            point = JointTrajectoryPoint()
            point.positions = arm_positions
            point.time_from_start = _duration(self._trajectory_time_sec)
            msg.points = [point]
            self._arm_publisher.publish(msg)

    def _solve_arm(self, runtime, target, now, dt):
        if target is None or now - target.arrival > self._pose_timeout_sec:
            runtime.was_stale = True
            self._warn(f'{runtime.side}_pose_stale', f'{runtime.side} VR pose is stale')
            return None
        transformed = self._transform_pose(target.message, runtime.chain.base_link, runtime.side)
        if transformed is None:
            runtime.was_stale = True
            return None
        if runtime.was_stale:
            runtime.filter.reset()
            runtime.was_stale = False
        
        filtered = runtime.filter.update(transformed, dt, self._position_tau, self._orientation_tau)
        filtered = self._clamp_workspace(filtered, runtime.chain)
        result = runtime.chain.solve(filtered, runtime.solution, **self._ik_options)
        runtime.solution = result.positions
        self._report_ik_residual(runtime, result)

        if result.position_error > self._max_ik_position_error:
            if runtime.command is None: return None
            return runtime.command
            
        if runtime.command is None:
            runtime.command = self._seed_for_chain(runtime.chain)
            
        max_delta = self._max_joint_velocity * dt
        delta = np.clip(result.positions - runtime.command, -max_delta, max_delta)
        runtime.command = np.clip(runtime.command + delta, runtime.chain.lower, runtime.chain.upper)
        return runtime.command

    def _transform_pose(self, message, target_frame, side):
        position = (message.pose.position.x, message.pose.position.y, message.pose.position.z)
        quaternion = (message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w)
        try:
            source_pose = pose_matrix(position, quaternion)
        except ValueError as exc:
            self._warn(f'{side}_bad_pose', f'{side} pose rejected: {exc}')
            return None
            
        source_frame = message.header.frame_id
        if not source_frame:
            return None
        if source_frame == target_frame:
            return source_pose
            
        try:
            transform = self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            self._warn(f'{side}_tf', f'No TF {target_frame} <- {source_frame}; holding arm: {exc}')
            return None
            
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        base_from_source = pose_matrix(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        return base_from_source @ source_pose

    def _clamp_workspace(self, target, chain):
        maximum = chain.nominal_reach * self._workspace_reach_scale
        radius = float(np.linalg.norm(target[:3, 3]))
        if radius > maximum:
            target = target.copy()
            target[:3, 3] *= maximum / radius
            self._warn(f'{chain.tip_link}_workspace', f'Target outside {chain.tip_link} reach; clamped to {maximum:.3f} m')
        return target

    def _report_ik_residual(self, runtime, result: IKResult):
        if result.converged: return
        self._warn(
            f'{runtime.side}_ik',
            f'{runtime.side} IK residual: position={result.position_error:.3f} m, '
            f'orientation={result.orientation_error:.3f} rad'
        )

    def _publish_hand_trajectories(self, now, dt):
        if self._joy_arrival is None or now - self._joy_arrival > self._joy_timeout_sec:
            return
        left = self._left_closure_filter.update(self._left_closure_target, dt, self._finger_tau)
        right = self._right_closure_filter.update(self._right_closure_target, dt, self._finger_tau)
        self._publish_hand(self._left_hand, left, self._left_hand_publisher)
        self._publish_hand(self._right_hand, right, self._right_hand_publisher)

    def _publish_hand(self, config, closure, publisher):
        if not config['names']: return
        positions = config['open'] + closure * (config['closed'] - config['open'])
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(config['names'])
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = _duration(self._trajectory_time_sec)
        message.points = [point]
        publisher.publish(message)

    def _is_enabled(self, now):
        return (
            self._teleop_active
            and self._active_arrival is not None
            and now - self._active_arrival <= self._active_timeout_sec
        )

    def _publish_zero_velocity(self):
        self._cmd_vel_publisher.publish(Twist())
        self._velocity_streaming = False

    def _warn(self, key, message):
        now = time.monotonic()
        if now - self._warning_times.get(key, 0.0) >= 2.0:
            self.get_logger().warning(message)
            self._warning_times[key] = now

    def stop(self):
        self._publish_zero_velocity()

def main(args=None):
    rclpy.init(args=args)
    node = R1KinematicsControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
