"""ROS 2 node for smoothed bimanual IK, fingers, and safe Gazebo velocity."""

from dataclasses import dataclass
import math
import os
import time
from typing import Dict, Optional, Sequence

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .body_proxy import (
    BodyCalibration,
    BodyProxyConfig,
    BodyProxyResult,
    BodyProxyTransformer,
)
from .filters import PoseEMA, ScalarEMA
from .kinematics import IKResult, SerialChain, urdf_joint_names
from .math3d import matrix_quaternion, pose_matrix
from .pose_range import ArmPoseRange


@dataclass
class PoseTarget:
    """The newest pose message and its local arrival time."""

    message: PoseStamped
    arrival: float


@dataclass
class ArmRuntime:
    """Mutable solver/filter state for one configured arm."""

    side: str
    chain: SerialChain
    filter: PoseEMA
    solution: np.ndarray
    command: Optional[np.ndarray] = None
    was_stale: bool = True
    last_recovery_attempt: float = -math.inf


class R1KinematicsControl(Node):
    """Convert VR intents to ros2_control trajectories and gated Gazebo velocity."""

    def __init__(self):
        super().__init__('r1_kinematics_control')
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._left_target: Optional[PoseTarget] = None
        self._right_target: Optional[PoseTarget] = None
        self._head_target: Optional[PoseTarget] = None
        self._arms: Dict[str, ArmRuntime] = {}
        self._joint_positions: Dict[str, float] = {}
        self._description_hash = None
        self._teleop_active = False
        self._active_seen = False
        self._active_arrival: Optional[float] = None
        self._velocity_arrival: Optional[float] = None
        self._velocity_streaming = False
        self._joy_arrival: Optional[float] = None
        self._left_closure_target = 0.0
        self._right_closure_target = 0.0
        self._left_closure_filter = ScalarEMA()
        self._right_closure_filter = ScalarEMA()
        self._hands_commanded = False
        self._enabled_last_tick = False
        self._body_neutral_checked = False
        self._arms_neutral_requested = False
        self._last_tick = time.monotonic()
        self._last_debug_log = 0.0
        self._warning_times: Dict[str, float] = {}

        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        description_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._arm_publisher = self.create_publisher(
            JointTrajectory, self._arm_command_topic, command_qos
        )
        self._debug_arm_publisher = self.create_publisher(
            JointTrajectory, f'{self._debug_topic_prefix}/arm_trajectory', command_qos
        )
        self._left_hand_publisher = self.create_publisher(
            JointTrajectory, self._left_hand_command_topic, command_qos
        )
        self._right_hand_publisher = self.create_publisher(
            JointTrajectory, self._right_hand_command_topic, command_qos
        )
        self._debug_left_hand_publisher = self.create_publisher(
            JointTrajectory,
            f'{self._debug_topic_prefix}/left_hand_trajectory',
            command_qos,
        )
        self._debug_right_hand_publisher = self.create_publisher(
            JointTrajectory,
            f'{self._debug_topic_prefix}/right_hand_trajectory',
            command_qos,
        )
        self._cmd_vel_publisher = self.create_publisher(
            Twist, self._cmd_vel_output_topic, command_qos
        )
        self._debug_cmd_vel_publisher = self.create_publisher(
            Twist, f'{self._debug_topic_prefix}/cmd_vel', command_qos
        )
        self._waist_hold_publisher = None
        self._debug_waist_hold_publisher = self.create_publisher(
            Float64MultiArray, f'{self._debug_topic_prefix}/waist_hold', command_qos
        )
        if self._waist_hold_enabled:
            self._waist_hold_publisher = self.create_publisher(
                Float64MultiArray, self._waist_hold_topic, command_qos
            )

        self._debug_pose_publishers = {}
        if self._debug_enabled:
            for name in (
                'head_world',
                'left_hand_world',
                'right_hand_world',
                'left_hand_local',
                'right_hand_local',
                'left_hand_body',
                'right_hand_body',
                'left_target',
                'right_target',
            ):
                self._debug_pose_publishers[name] = self.create_publisher(
                    PoseStamped, f'{self._debug_topic_prefix}/{name}', qos_profile_sensor_data
                )

        self.create_subscription(
            PoseStamped,
            self._left_pose_topic,
            self._left_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self._right_pose_topic,
            self._right_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self._head_pose_topic,
            self._head_pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Joy, self._joy_topic, self._joy_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            self._cmd_vel_input_topic,
            self._velocity_callback,
            command_qos,
        )
        self.create_subscription(
            Bool, self._active_topic, self._active_callback, command_qos
        )
        self.create_subscription(
            Bool,
            self._arms_neutral_topic,
            self._arms_neutral_callback,
            command_qos,
        )
        self.create_subscription(
            String,
            self._robot_description_topic,
            self._description_callback,
            description_qos,
        )
        self.create_service(
            Trigger,
            self._body_calibration_service,
            self._calibrate_body_callback,
        )

        self._control_timer = self.create_timer(
            1.0 / self._control_rate_hz, self._control_tick
        )

        self._load_initial_description()
        self._load_body_calibration()
        self._arm_range_probe = None
        self._arm_range_probe_targets = {}
        probe_file = str(self.get_parameter('arm_range_probe_file').value)
        self._arm_range_probe_fraction = float(
            self.get_parameter('arm_range_probe_fraction').value)
        if probe_file:
            if (
                not self._headset_relative_enabled
                or set(self._arms) != {'left', 'right'}
            ):
                raise ValueError(
                    'arm range probe requires calibrated body mapping and both URDF chains'
                )
            self._arm_range_probe = ArmPoseRange.load(
                os.path.expanduser(probe_file),
                {side: runtime.chain for side, runtime in self._arms.items()})
            if not 0.0 < self._arm_range_probe_fraction <= (
                self._arm_range_probe.max_probe_fraction
            ):
                raise ValueError('arm range probe exceeds validated profile limit')
            self.get_logger().warning(
                f'Supervised arm range probe enabled: {self._arm_range_probe_fraction:.0%}; '
                f'rate capped at '
                f'{min(self._max_joint_velocity, self._arm_range_probe.max_probe_rate):.2f} rad/s'
            )
        self.get_logger().info(
            'R1 kinematics control started in safe state; '
            f'headset_relative={self._headset_relative_enabled}; '
            f'dry_run={self._dry_run}; waiting for deadman'
        )

    def _declare_parameters(self):
        self.declare_parameter('arm_range_probe_file', '')
        self.declare_parameter('arm_range_probe_fraction', 0.10)
        self.declare_parameter('control_rate_hz', 60.0)
        self.declare_parameter('pose_timeout_sec', 0.25)
        self.declare_parameter('joy_timeout_sec', 0.5)
        self.declare_parameter('active_timeout_sec', 1.5)
        self.declare_parameter('cmd_vel_timeout_sec', 0.25)
        self.declare_parameter('position_ema_tau_sec', 0.075)
        self.declare_parameter('orientation_ema_tau_sec', 0.10)
        self.declare_parameter('finger_ema_tau_sec', 0.12)
        self.declare_parameter('trajectory_time_sec', 0.10)
        self.declare_parameter('max_joint_velocity_rad_s', 1.0)
        self.declare_parameter('workspace_reach_scale', 0.98)
        self.declare_parameter('max_ik_position_error_m', 0.08)
        self.declare_parameter('ik.max_iterations', 30)
        self.declare_parameter('ik.damping', 0.03)
        self.declare_parameter('ik.singularity_threshold', 0.05)
        self.declare_parameter('ik.max_joint_step_rad', 0.15)
        self.declare_parameter('ik.position_tolerance_m', 0.005)
        self.declare_parameter('ik.orientation_tolerance_rad', 0.04)
        self.declare_parameter('ik.orientation_weight', 0.35)
        # A numerical IK solve can settle in a bad local minimum after a
        # tracking jump (most visible on the mirrored left shoulder).  A
        # throttled neutral-seed retry recovers without bypassing limits.
        self.declare_parameter('ik.recovery_enabled', True)
        self.declare_parameter('ik.recovery_min_improvement_m', 0.01)
        self.declare_parameter('ik.recovery_retry_sec', 0.50)

        self.declare_parameter('robot_description', '')
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('robot_description_topic', '/robot_description')
        self.declare_parameter('headset_relative_enabled', True)
        self.declare_parameter('dry_run', True)
        # Non-dry-run output is reserved for the localhost-only Gazebo launch.
        # This node is never a physical R1 writer.
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter(
            'debug_topic_prefix', '/r1_kinematics_control/debug'
        )
        self.declare_parameter('debug_log_period_sec', 1.0)
        self.declare_parameter('pose_calibration_frame', 'vr_tracking')
        self.declare_parameter('head_pose_topic', '/vr/head/pose')
        self.declare_parameter('body_proxy.require_calibration', True)
        self.declare_parameter(
            'body_proxy.calibration_service', '/vr/calibrate_body'
        )
        self.declare_parameter(
            'body_proxy.calibration_file',
            '~/Unitree_Project/Configs/Calibration/r1_body_calibration.json',
        )
        # A saved calibration must match the operator's starting neutral pose.
        # Without this check a stale controller offset can reach the IK solver.
        self.declare_parameter('body_proxy.neutral_pose_guard_enabled', True)
        self.declare_parameter('exhibition_session_mode', False)
        self.declare_parameter('body_proxy.neutral_pose_max_error_m', 0.15)
        self.declare_parameter('body_proxy.sync_tolerance_sec', 0.08)
        self.declare_parameter('body_proxy.user_height_m', 0.0)
        self.declare_parameter('body_proxy.fallback_user_height_m', 1.75)
        self.declare_parameter('body_proxy.shoulder_width_m', 0.40)
        self.declare_parameter('body_proxy.user_arm_reach_m', 0.65)
        self.declare_parameter('body_proxy.head_height_ratio', 0.93)
        self.declare_parameter('body_proxy.neck_height_ratio', 0.86)
        self.declare_parameter('body_proxy.shoulder_height_ratio', 0.82)
        self.declare_parameter('body_proxy.chest_height_ratio', 0.72)
        self.declare_parameter('body_proxy.waist_height_ratio', 0.53)
        self.declare_parameter('body_proxy.shoulder_height_offset_m', 0.0)
        self.declare_parameter('body_proxy.shoulder_forward_offset_m', -0.02)
        self.declare_parameter('body_proxy.body_position_tau_sec', 0.12)
        self.declare_parameter('body_proxy.body_yaw_tau_sec', 0.20)
        self.declare_parameter('body_proxy.follow_head_position', False)
        self.declare_parameter('body_proxy.follow_head_yaw', False)
        self.declare_parameter('body_proxy.motion_scale', 1.0)
        self.declare_parameter('body_proxy.robot_reach_scale', 0.95)
        self.declare_parameter('body_proxy.max_behind_shoulder_m', 0.04)
        self.declare_parameter('body_proxy.min_hand_lateral_m', 0.08)
        self.declare_parameter('body_proxy.max_input_jump_m', 0.25)
        self.declare_parameter('body_proxy.max_target_speed_mps', 0.80)
        self.declare_parameter('left_arm.base_link', 'torso_link')
        self.declare_parameter('left_arm.tip_link', 'left_hand_link')
        self.declare_parameter('left_arm.joint_names', ['AUTO'])
        self.declare_parameter('right_arm.base_link', 'torso_link')
        self.declare_parameter('right_arm.tip_link', 'right_hand_link')
        self.declare_parameter('right_arm.joint_names', ['AUTO'])
        self.declare_parameter('left_pose_neutral_vr', [0.0, 0.0, 0.0])
        self.declare_parameter('right_pose_neutral_vr', [0.0, 0.0, 0.0])
        self.declare_parameter('left_pose_neutral_base', [0.0, 0.0, 0.0])
        self.declare_parameter('right_pose_neutral_base', [0.0, 0.0, 0.0])
        self.declare_parameter('left_pose_scale', [1.0, 1.0, 1.0])
        self.declare_parameter('right_pose_scale', [1.0, 1.0, 1.0])

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
        self.declare_parameter(
            'arms_neutral_topic', '/vr/actions/arms_neutral'
        )
        self.declare_parameter('cmd_vel_input_topic', '/vr/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel')
        # Simulation-only hold for the otherwise uncommanded waist joints.
        # Leave disabled for hardware deployments unless a matching controller
        # and safety policy are explicitly configured.
        self.declare_parameter('waist_hold_enabled', False)
        self.declare_parameter(
            'waist_hold_topic', '/waist_hold_controller/commands'
        )
        self.declare_parameter('waist_hold_positions', [0.0, 0.0])
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter(
            'arm_command_topic', '/joint_trajectory_controller/joint_trajectory'
        )
        self.declare_parameter(
            'left_hand_command_topic', '/left_hand_controller/joint_trajectory'
        )
        self.declare_parameter(
            'right_hand_command_topic', '/right_hand_controller/joint_trajectory'
        )
        self.declare_parameter('max_forward_mps', 0.35)
        self.declare_parameter('max_lateral_mps', 0.25)
        self.declare_parameter('max_yaw_rps', 0.60)

    def _read_parameters(self):
        def value(name):
            return self.get_parameter(name).value

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
        self._ik_recovery_enabled = bool(value('ik.recovery_enabled'))
        self._ik_recovery_min_improvement = float(
            value('ik.recovery_min_improvement_m')
        )
        self._ik_recovery_retry_sec = float(value('ik.recovery_retry_sec'))
        self._robot_description = str(value('robot_description'))
        self._urdf_path = str(value('urdf_path'))
        self._robot_description_topic = str(value('robot_description_topic'))
        self._headset_relative_enabled = bool(
            value('headset_relative_enabled')
        )
        self._dry_run = bool(value('dry_run'))
        self._simulation_mode = bool(value('simulation_mode'))
        self._debug_enabled = bool(value('debug_enabled'))
        self._debug_topic_prefix = str(value('debug_topic_prefix')).rstrip('/')
        self._debug_log_period = float(value('debug_log_period_sec'))
        self._pose_calibration_frame = str(value('pose_calibration_frame'))
        self._head_pose_topic = str(value('head_pose_topic'))
        self._body_require_calibration = bool(
            value('body_proxy.require_calibration')
        )
        self._body_calibration_service = str(
            value('body_proxy.calibration_service')
        )
        self._body_calibration_file = str(value('body_proxy.calibration_file'))
        self._body_neutral_guard_enabled = bool(
            value('body_proxy.neutral_pose_guard_enabled')
        )
        self._exhibition_session_mode = bool(value('exhibition_session_mode'))
        self._body_neutral_max_error = float(
            value('body_proxy.neutral_pose_max_error_m')
        )
        self._body_sync_tolerance = float(
            value('body_proxy.sync_tolerance_sec')
        )
        self._body_proxy_config = BodyProxyConfig(
            user_height_m=float(value('body_proxy.user_height_m')),
            fallback_user_height_m=float(
                value('body_proxy.fallback_user_height_m')
            ),
            shoulder_width_m=float(value('body_proxy.shoulder_width_m')),
            user_arm_reach_m=float(value('body_proxy.user_arm_reach_m')),
            head_height_ratio=float(value('body_proxy.head_height_ratio')),
            neck_height_ratio=float(value('body_proxy.neck_height_ratio')),
            shoulder_height_ratio=float(
                value('body_proxy.shoulder_height_ratio')
            ),
            chest_height_ratio=float(value('body_proxy.chest_height_ratio')),
            waist_height_ratio=float(value('body_proxy.waist_height_ratio')),
            shoulder_height_offset_m=float(
                value('body_proxy.shoulder_height_offset_m')
            ),
            shoulder_forward_offset_m=float(
                value('body_proxy.shoulder_forward_offset_m')
            ),
            body_position_tau_sec=float(
                value('body_proxy.body_position_tau_sec')
            ),
            body_yaw_tau_sec=float(value('body_proxy.body_yaw_tau_sec')),
            follow_head_position=bool(
                value('body_proxy.follow_head_position')
            ),
            follow_head_yaw=bool(value('body_proxy.follow_head_yaw')),
            motion_scale=float(value('body_proxy.motion_scale')),
            robot_reach_scale=float(value('body_proxy.robot_reach_scale')),
            max_behind_shoulder_m=float(
                value('body_proxy.max_behind_shoulder_m')
            ),
            min_hand_lateral_m=float(value('body_proxy.min_hand_lateral_m')),
            max_input_jump_m=float(value('body_proxy.max_input_jump_m')),
            max_target_speed_mps=float(
                value('body_proxy.max_target_speed_mps')
            ),
        )
        self._body_proxy = BodyProxyTransformer(self._body_proxy_config)
        self._pose_calibration = {}
        for side in ('left', 'right'):
            self._pose_calibration[side] = {
                'neutral_vr': np.asarray(
                    value(f'{side}_pose_neutral_vr'), dtype=float
                ),
                'neutral_base': np.asarray(
                    value(f'{side}_pose_neutral_base'), dtype=float
                ),
                'scale': np.asarray(value(f'{side}_pose_scale'), dtype=float),
            }
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
        self._arms_neutral_topic = str(value('arms_neutral_topic'))
        self._cmd_vel_input_topic = str(value('cmd_vel_input_topic'))
        self._cmd_vel_output_topic = str(value('cmd_vel_output_topic'))
        self._waist_hold_enabled = bool(value('waist_hold_enabled'))
        self._waist_hold_topic = str(value('waist_hold_topic'))
        self._waist_hold_positions = [
            float(position) for position in value('waist_hold_positions')
        ]
        self._joint_states_topic = str(value('joint_states_topic'))
        self._arm_command_topic = str(value('arm_command_topic'))
        self._left_hand_command_topic = str(value('left_hand_command_topic'))
        self._right_hand_command_topic = str(value('right_hand_command_topic'))
        self._max_forward = float(value('max_forward_mps'))
        self._max_lateral = float(value('max_lateral_mps'))
        self._max_yaw = float(value('max_yaw_rps'))

    def _hand_config(self, side):
        def value(suffix):
            return self.get_parameter(f'{side}_hand.{suffix}').value

        names = _optional_names(value('joint_names'), 'DISABLED')
        opened = np.asarray(value('open_positions'), dtype=float)
        closed = np.asarray(value('closed_positions'), dtype=float)
        return {'names': names, 'open': opened, 'closed': closed}

    def _validate_parameters(self):
        if not self._dry_run and not self._simulation_mode:
            raise RuntimeError(
                'r1_kinematics_control may publish controller topics only in '
                'the isolated Gazebo simulation_mode; use dry_run=true outside it'
            )
        if self._control_rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        if min(
            self._pose_timeout_sec,
            self._joy_timeout_sec,
            self._active_timeout_sec,
            self._cmd_vel_timeout_sec,
            self._trajectory_time_sec,
            self._max_joint_velocity,
        ) <= 0.0:
            raise ValueError('timeouts, trajectory time, and speed must be positive')
        if not 0.0 < self._workspace_reach_scale <= 1.0:
            raise ValueError('workspace_reach_scale must be in (0, 1]')
        if self._debug_log_period <= 0.0:
            raise ValueError('debug_log_period_sec must be positive')
        if self._ik_recovery_min_improvement < 0.0:
            raise ValueError('ik.recovery_min_improvement_m must be non-negative')
        if self._ik_recovery_retry_sec <= 0.0:
            raise ValueError('ik.recovery_retry_sec must be positive')
        if (
            not math.isfinite(self._body_sync_tolerance)
            or self._body_sync_tolerance <= 0.0
            or self._body_sync_tolerance > 0.10
        ):
            raise ValueError(
                'body_proxy.sync_tolerance_sec must be within (0, 0.10]'
            )
        if (
            not math.isfinite(self._body_neutral_max_error)
            or self._body_neutral_max_error <= 0.0
            or self._body_neutral_max_error > 0.50
        ):
            raise ValueError(
                'body_proxy.neutral_pose_max_error_m must be within (0, 0.50]'
            )
        if not self._debug_topic_prefix.startswith('/'):
            raise ValueError('debug_topic_prefix must be an absolute ROS topic')
        if not self._head_pose_topic or not self._body_calibration_service:
            raise ValueError('head pose topic and body calibration service are required')
        if not self._arms_neutral_topic.startswith('/'):
            raise ValueError('arms_neutral_topic must be an absolute ROS topic')
        if math.isclose(self._trigger_released, self._trigger_pressed):
            raise ValueError('trigger released and pressed values must differ')
        if self._waist_hold_enabled:
            if len(self._waist_hold_positions) != 2 or not all(
                math.isfinite(position) for position in self._waist_hold_positions
            ):
                raise ValueError(
                    'waist_hold_positions must contain two finite values'
                )
        for side, hand in (('left', self._left_hand), ('right', self._right_hand)):
            if not hand['names']:
                continue
            size = len(hand['names'])
            if hand['open'].shape != (size,) or hand['closed'].shape != (size,):
                raise ValueError(
                    f'{side} hand names/open/closed arrays must have equal lengths'
                )
        if not self._pose_calibration_frame:
            raise ValueError('pose_calibration_frame must not be empty')
        for side, calibration in self._pose_calibration.items():
            for name in ('neutral_vr', 'neutral_base', 'scale'):
                vector = calibration[name]
                if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                    raise ValueError(
                        f'{side}_pose_{name} must contain three finite numbers'
                    )
            if np.any(np.abs(calibration['scale']) < 1.0e-6):
                raise ValueError(f'{side}_pose_scale values must be non-zero')

    def _load_initial_description(self):
        if self._robot_description.strip():
            self._configure_chains(self._robot_description)
            return
        if not self._urdf_path:
            self.get_logger().warning(
                f'No URDF loaded; waiting on {self._robot_description_topic}'
            )
            return
        path = os.path.abspath(os.path.expanduser(self._urdf_path))
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                self._configure_chains(stream.read())
        except OSError as exc:
            self.get_logger().error(f'Cannot read URDF {path!r}: {exc}')

    def _load_body_calibration(self):
        if not self._headset_relative_enabled or not self._body_calibration_file:
            return
        path = os.path.abspath(os.path.expanduser(self._body_calibration_file))
        if not os.path.exists(path):
            self.get_logger().info(
                f'No saved body calibration at {path!r}; '
                f'call {self._body_calibration_service} with deadman released'
            )
            return
        try:
            calibration = BodyCalibration.load(path)
            self._body_proxy.use_calibration(calibration)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(
                f'Cannot load body calibration {path!r}: {exc}'
            )
            return
        self.get_logger().info(
            f'Loaded headset-relative body calibration from {path!r}'
        )

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
                chain = SerialChain.from_urdf(
                    description,
                    config['base'],
                    config['tip'],
                    config['joints'],
                )
                seed = self._seed_for_chain(chain)
                configured[side] = ArmRuntime(
                    side=side,
                    chain=chain,
                    filter=PoseEMA(),
                    solution=seed,
                )
            names = [
                name
                for runtime in configured.values()
                for name in runtime.chain.joint_names
            ]
            if len(names) != len(set(names)):
                raise ValueError('left and right arm chains contain duplicate joints')
            available_joints = urdf_joint_names(description)
            for side, hand in (
                ('left', self._left_hand),
                ('right', self._right_hand),
            ):
                missing = set(hand['names'] or ()) - available_joints
                if missing:
                    raise ValueError(
                        f'{side} hand joints are absent from URDF: {sorted(missing)}'
                    )
        except ValueError as exc:
            self.get_logger().error(f'URDF chain configuration rejected: {exc}')
            return

        self._arms = configured
        self._description_hash = description_hash
        if self._headset_relative_enabled:
            left_chain = self._arms['left'].chain
            right_chain = self._arms['right'].chain
            if left_chain.base_link != right_chain.base_link:
                self.get_logger().error(
                    'Headset-relative mode requires a common arm base frame; '
                    f'got {left_chain.base_link!r} and {right_chain.base_link!r}'
                )
                self._arms = {}
                self._description_hash = None
                return
            try:
                self._body_proxy.set_robot_geometry(
                    left_chain.proximal_joint_position(),
                    right_chain.proximal_joint_position(),
                    left_chain.forward(left_chain.neutral_positions()),
                    right_chain.forward(right_chain.neutral_positions()),
                    left_chain.distal_reach,
                    right_chain.distal_reach,
                )
            except ValueError as exc:
                self.get_logger().error(f'Body proxy geometry rejected: {exc}')
                self._arms = {}
                self._description_hash = None
                return
        for side, runtime in self._arms.items():
            self.get_logger().info(
                f'{side} IK chain: {runtime.chain.base_link} -> '
                f'{runtime.chain.tip_link}; joints={runtime.chain.joint_names}; '
                f'nominal reach={runtime.chain.nominal_reach:.3f} m'
            )

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

    def _head_pose_callback(self, message):
        self._head_target = PoseTarget(message=message, arrival=time.monotonic())

    def _calibrate_body_callback(self, _request, response):
        now = time.monotonic()
        if self._is_enabled(now):
            response.success = False
            response.message = 'Release deadman before body calibration'
            return response
        poses = self._fresh_tracking_poses(now, warn=False)
        if poses is None:
            issue = self._tracking_pose_issue(now)
            detail = (
                issue[1]
                if issue is not None
                else 'one or more poses has an invalid transform'
            )
            response.success = False
            response.message = (
                'Fresh synchronized head, left hand, and right hand poses are '
                f'required ({detail})'
            )
            return response
        calibration = self._body_proxy.calibrate(
            poses['head'], poses['left'], poses['right']
        )
        saved = 'memory only'
        if self._body_calibration_file:
            path = os.path.abspath(os.path.expanduser(self._body_calibration_file))
            try:
                calibration.save(path)
                saved = path
            except OSError as exc:
                response.success = False
                response.message = f'Calibration captured but not saved: {exc}'
                return response
        for runtime in self._arms.values():
            runtime.filter.reset()
            runtime.was_stale = True
        self._body_neutral_checked = False
        response.success = True
        response.message = (
            f'Calibrated: head_height={calibration.head_height_m:.3f} m, '
            f'user_height={calibration.user_height_m:.3f} m, '
            f'shoulder_width={calibration.shoulder_width_m:.3f} m, '
            f'arm_reach={calibration.user_arm_reach_m:.3f} m, saved={saved}'
        )
        self.get_logger().info(response.message)
        return response

    def _joint_state_callback(self, message):
        if len(message.name) != len(message.position):
            self._warn('joint_state_size', 'Rejected malformed JointState')
            return
        for name, position in zip(message.name, message.position):
            if math.isfinite(position):
                self._joint_positions[name] = float(position)

    def _active_callback(self, message):
        self._active_arrival = time.monotonic()
        active = bool(message.data)
        transitioned = not self._active_seen or active != self._teleop_active
        self._active_seen = True
        self._teleop_active = active
        if not active and transitioned:
            self._publish_zero_velocity()
            self._publish_hold_trajectories()
            self._body_proxy.reset_filter()
            if not self._exhibition_session_mode:
                self._body_neutral_checked = False
            for runtime in self._arms.values():
                runtime.was_stale = True
            self._enabled_last_tick = False

    def _arms_neutral_callback(self, message):
        requested = bool(message.data)
        if requested == self._arms_neutral_requested:
            return
        self._arms_neutral_requested = requested
        if requested:
            self.get_logger().info('arms_reset_to_neutral')
        else:
            self.get_logger().info('arms VR-follow restored')

    def _joy_callback(self, message):
        self._joy_arrival = time.monotonic()
        self._left_closure_target = self._trigger_value(
            message, self._left_trigger_axis, self._left_trigger_button, 'left'
        )
        self._right_closure_target = self._trigger_value(
            message, self._right_trigger_axis, self._right_trigger_button, 'right'
        )

    def _trigger_value(self, message, axis_index, button_index, side):
        if axis_index >= 0 and axis_index < len(message.axes):
            raw = float(message.axes[axis_index])
            normalized = (
                (raw - self._trigger_released)
                / (self._trigger_pressed - self._trigger_released)
            )
            return max(0.0, min(1.0, normalized))
        if button_index >= 0 and button_index < len(message.buttons):
            return 1.0 if message.buttons[button_index] else 0.0
        self._warn(
            f'{side}_trigger_missing',
            f'{side} trigger index is absent from /vr/joy; using open hand',
        )
        return 0.0

    def _velocity_callback(self, message):
        self._velocity_arrival = time.monotonic()
        if not self._is_enabled(self._velocity_arrival):
            return
        values = (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            self._warn('invalid_velocity', 'Non-finite /vr/cmd_vel was rejected')
            self._publish_zero_velocity()
            return
        output = Twist()
        output.linear.x = _clamp(message.twist.linear.x, self._max_forward)
        output.linear.y = _clamp(message.twist.linear.y, self._max_lateral)
        output.angular.z = _clamp(message.twist.angular.z, self._max_yaw)
        self._publish_velocity_command(output)
        self._velocity_streaming = True

    def _control_tick(self):
        now = time.monotonic()
        dt = min(0.1, max(1.0e-4, now - self._last_tick))
        self._last_tick = now
        enabled = self._is_enabled(now)

        if self._waist_hold_publisher is not None:
            hold = Float64MultiArray()
            hold.data = list(self._waist_hold_positions)
            self._debug_waist_hold_publisher.publish(hold)
            if not self._dry_run:
                self._waist_hold_publisher.publish(hold)

        if self._enabled_last_tick and not enabled:
            self._publish_hold_trajectories()
        self._enabled_last_tick = enabled

        velocity_stale = (
            self._velocity_arrival is None
            or now - self._velocity_arrival > self._cmd_vel_timeout_sec
        )
        if self._velocity_streaming and (not enabled or velocity_stale):
            self._publish_zero_velocity()

        if not enabled:
            # Commissioning checks neutral at each Deadman press. Exhibition
            # LOCK/reconnect keeps a previously verified calibration: the
            # operator may have legitimately moved since the first RUN.
            # Explicit body calibration still resets the verification flag.
            if not self._exhibition_session_mode:
                self._body_neutral_checked = False
            return
        self._publish_arm_trajectory(now, dt)
        self._publish_hand_trajectories(now, dt)

    def _publish_arm_trajectory(self, now, dt):
        if set(self._arms) != {'left', 'right'}:
            return
        targets = self._arm_target_poses(now, dt)
        if targets is None:
            return
        outputs = []
        for side in ('left', 'right'):
            output = self._solve_arm(self._arms[side], targets[side], dt)
            if output is None:
                return
            outputs.append((self._arms[side], output))

        message = JointTrajectory()
        # A zero stamp means "start immediately".  Using the current Gazebo
        # sim-time here can become stale before controller_manager receives the
        # message when the simulator runs faster than wall time.
        message.joint_names = [
            name for runtime, _ in outputs for name in runtime.chain.joint_names
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            float(value) for _, output in outputs for value in output
        ]
        point.time_from_start = _duration(self._trajectory_time_sec)
        message.points = [point]
        self._publish_arm_command(message)

    def _publish_arm_command(self, message):
        self._debug_arm_publisher.publish(message)
        if not self._dry_run:
            self._arm_publisher.publish(message)

    def _solve_arm(self, runtime, transformed, dt):
        if self._arm_range_probe is not None:
            target = self._arm_range_probe_targets.get(runtime.side)
            if target is None:
                return None
            # First frame must be model-neutral for the physical writer's
            # independent feedback-relative takeover. Do not seed this preview
            # from physical feedback and then drift toward model-zero.
            if runtime.command is None:
                runtime.command = (
                    self._arm_range_probe.reset_target(runtime.side)
                    if self._arm_range_probe.absolute_targets
                    else runtime.chain.neutral_positions())
                return runtime.command.copy()
            step = (
                min(self._max_joint_velocity, self._arm_range_probe.max_probe_rate)
                * max(0.0, min(dt, .05))
            )
            delta = target - runtime.command
            # A common interpolation factor preserves the convex pose range.
            distance = float(np.max(np.abs(delta)))
            candidate = runtime.command + delta * min(1.0, step / max(distance, 1e-12))
            candidate = np.clip(candidate, runtime.chain.lower, runtime.chain.upper)
            if not self._arm_range_probe.contains_target(runtime.side, candidate):
                self._warn(
                    'probe_envelope_' + runtime.side,
                    'Arm range target held at configured envelope',
                )
                return runtime.command.copy()
            runtime.command = candidate
            runtime.solution = runtime.command.copy()
            return runtime.command.copy()
        if runtime.was_stale:
            runtime.filter.reset()
            runtime.was_stale = False
        filtered = runtime.filter.update(
            transformed, dt, self._position_tau, self._orientation_tau
        )
        filtered = self._clamp_workspace(filtered, runtime.chain)
        # Seed from the actually commanded pose once the arm is running.  The
        # raw solver result may jump to another elbow branch when a VR sample
        # is noisy; feeding that jump back as the next seed is a common source
        # of visible chatter.  ``runtime.command`` is velocity-limited below,
        # so it provides a continuous branch anchor while still allowing the
        # neutral-seed recovery path to escape a genuine local minimum.
        solver_seed = (
            runtime.command
            if runtime.command is not None
            else runtime.solution
        )
        result = runtime.chain.solve(filtered, solver_seed, **self._ik_options)
        # Recover from a local minimum using a deterministic neutral seed.
        # This path is only entered for a large residual and is throttled so a
        # permanently unreachable target cannot consume the control loop.
        now = time.monotonic()
        if (
            self._ik_recovery_enabled
            and result.position_error > self._max_ik_position_error
            and now - runtime.last_recovery_attempt >= self._ik_recovery_retry_sec
        ):
            runtime.last_recovery_attempt = now
            recovery = runtime.chain.solve(
                filtered,
                runtime.chain.neutral_positions(),
                **self._ik_options,
            )
            if (
                recovery.position_error
                + self._ik_recovery_min_improvement
                < result.position_error
            ):
                self._warn(
                    f'{runtime.side}_ik_recovery',
                    f'{runtime.side} IK reseeded from neutral: '
                    f'{result.position_error:.3f} -> '
                    f'{recovery.position_error:.3f} m',
                )
                result = recovery
        runtime.solution = result.positions
        self._report_ik_residual(runtime, result)

        if result.position_error > self._max_ik_position_error:
            if runtime.command is None:
                return None
            return runtime.command
        if runtime.command is None:
            runtime.command = self._seed_for_chain(runtime.chain)
        max_delta = self._max_joint_velocity * dt
        delta = np.clip(result.positions - runtime.command, -max_delta, max_delta)
        runtime.command = np.clip(
            runtime.command + delta, runtime.chain.lower, runtime.chain.upper
        )
        return runtime.command

    def _arm_target_poses(self, now, dt):
        if self._headset_relative_enabled:
            return self._headset_relative_targets(now, dt)
        targets = {}
        for side, pose_target in (
            ('left', self._left_target),
            ('right', self._right_target),
        ):
            if pose_target is None or now - pose_target.arrival > self._pose_timeout_sec:
                self._arms[side].was_stale = True
                self._warn(f'{side}_pose_stale', f'{side} VR pose is stale')
                return None
            transformed = self._legacy_transform_pose(
                pose_target.message, self._arms[side].chain.base_link, side
            )
            if transformed is None:
                self._arms[side].was_stale = True
                return None
            targets[side] = transformed
        return targets

    def _headset_relative_targets(self, now, dt):
        self._arm_range_probe_targets = {}
        poses = self._fresh_tracking_poses(now, warn=True)
        if poses is None:
            for runtime in self._arms.values():
                runtime.was_stale = True
            return None
        if self._body_proxy.calibration is None:
            if self._body_require_calibration:
                self._warn(
                    'body_not_calibrated',
                    'Headset-relative body is not calibrated; release deadman and '
                    f'call {self._body_calibration_service}',
                )
                return None
            calibration = self._body_proxy.calibrate(
                poses['head'], poses['left'], poses['right']
            )
            self.get_logger().warning(
                'Body proxy auto-calibrated from the first valid pose; '
                f'head_height={calibration.head_height_m:.3f} m'
            )
        try:
            result = self._body_proxy.update(
                poses['head'], poses['left'], poses['right'], dt
            )
        except (RuntimeError, ValueError) as exc:
            self._warn('body_proxy_update', f'Body proxy rejected poses: {exc}')
            return None
        self._publish_body_debug(result, now)
        if (
            self._body_neutral_guard_enabled
            and not self._body_neutral_checked
        ):
            errors = self._body_proxy.neutral_position_errors(
                result.hands_body_local
            )
            largest_side = max(errors, key=errors.get)
            largest_error = errors[largest_side]
            rest_start = False
            if largest_error > self._body_neutral_max_error and self._arm_range_probe is not None:
                rest_start = self._arm_range_probe.allows_rest_start({
                    side: result.hands_body_local[side][:3, 3] - getattr(
                        self._body_proxy.calibration, side + '_neutral_body')[:3, 3]
                    for side in ('left', 'right')})
            if largest_error > self._body_neutral_max_error and not rest_start:
                for runtime in self._arms.values():
                    runtime.was_stale = True
                self._warn(
                    'body_calibration_neutral_mismatch',
                    f'Starting pose differs from saved body calibration: '
                    f'{largest_side} error={largest_error:.3f} m > '
                    f'{self._body_neutral_max_error:.3f} m; release deadman '
                    f'and call {self._body_calibration_service}',
                )
                return None
            self._body_neutral_checked = True
        for side, reasons in result.clamped.items():
            if reasons:
                self._warn(
                    f'{side}_body_clamp',
                    f'{side} body target limited by {", ".join(reasons)}',
                )
        if self._arms_neutral_requested:
            if self._arm_range_probe is not None:
                self._arm_range_probe_targets = {
                    side: self._arm_range_probe.reset_target(side)
                    for side, runtime in self._arms.items()}
            # PoseEMA plus the independent joint-velocity limit below make
            # both the transition to neutral and the return to VR-follow
            # continuous.  The teleop/session latch remains unchanged.
            return self._body_proxy.neutral_targets()
        if self._arm_range_probe is not None:
            for side in ('left', 'right'):
                neutral = getattr(self._body_proxy.calibration, side + '_neutral_body')
                delta = result.hands_body_local[side][:3, 3] - neutral[:3, 3]
                self._arm_range_probe_targets[side] = self._arm_range_probe.probe_target(
                    side, delta, self._arm_range_probe_fraction)
        return result.targets

    def _fresh_tracking_poses(self, now, warn):
        issue = self._tracking_pose_issue(now)
        if issue is not None:
            kind, detail = issue
            if warn:
                self._warn(
                    f'body_pose_{kind}',
                    f'Headset-relative poses rejected: {detail}; holding both arms',
                )
            return None

        targets = {
            'head': self._head_target,
            'left': self._left_target,
            'right': self._right_target,
        }
        poses = {}
        for name, target in targets.items():
            pose = self._pose_in_frame(
                target.message, self._pose_calibration_frame, name
            )
            if pose is None:
                return None
            poses[name] = pose
        return poses

    def _tracking_pose_issue(self, now):
        """Return a concise reason when the three tracking poses are unusable."""
        targets = {
            'head': self._head_target,
            'left': self._left_target,
            'right': self._right_target,
        }
        stale = []
        for name, target in targets.items():
            if target is None:
                stale.append(f'{name}=missing')
                continue
            age = now - target.arrival
            if age > self._pose_timeout_sec:
                stale.append(f'{name}=stale({max(0.0, age):.2f}s)')
        if stale:
            return 'stale', ', '.join(stale)

        arrivals = [target.arrival for target in targets.values()]
        spread = max(arrivals) - min(arrivals)
        if spread > self._body_sync_tolerance:
            return (
                'sync',
                f'timestamp spread {spread:.3f}s exceeds '
                f'{self._body_sync_tolerance:.3f}s',
            )
        return None

    def _legacy_transform_pose(self, message, target_frame, side):
        base_pose = self._pose_in_frame(message, target_frame, side)
        if base_pose is None:
            return None
        if message.header.frame_id == self._pose_calibration_frame:
            calibration = self._pose_calibration[side]
            base_pose = base_pose.copy()
            base_pose[:3, 3] = calibration['neutral_base'] + calibration[
                'scale'
            ] * (base_pose[:3, 3] - calibration['neutral_vr'])
        return base_pose

    def _pose_in_frame(self, message, target_frame, label):
        position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        quaternion = (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        try:
            source_pose = pose_matrix(position, quaternion)
        except ValueError as exc:
            self._warn(f'{label}_bad_pose', f'{label} pose rejected: {exc}')
            return None
        source_frame = message.header.frame_id
        if not source_frame:
            self._warn(f'{label}_no_frame', f'{label} pose has an empty frame_id')
            return None
        if source_frame == target_frame:
            return source_pose
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, Time()
            )
        except TransformException as exc:
            self._warn(
                f'{label}_tf',
                f'No TF {target_frame} <- {source_frame}; holding arms: {exc}',
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        target_from_source = pose_matrix(
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        return target_from_source @ source_pose

    def _publish_body_debug(self, result: BodyProxyResult, now):
        if not self._debug_enabled:
            return
        common_frame = self._pose_calibration_frame
        self._publish_debug_pose('head_world', result.head_world, common_frame)
        for side in ('left', 'right'):
            self._publish_debug_pose(
                f'{side}_hand_world', result.hands_world[side], common_frame
            )
            self._publish_debug_pose(
                f'{side}_hand_local',
                result.hands_head_local[side],
                'vr_head_local',
            )
            self._publish_debug_pose(
                f'{side}_hand_body',
                result.hands_body_local[side],
                'vr_body_proxy',
            )
            self._publish_debug_pose(
                f'{side}_target',
                result.targets[side],
                self._arms[side].chain.base_link,
            )
        if now - self._last_debug_log < self._debug_log_period:
            return
        self._last_debug_log = now

        def xyz(transform):
            return np.round(transform[:3, 3], 3).tolist()

        self.get_logger().info(
            'body_proxy '
            f'head_pose={xyz(result.head_world)} '
            f'left_hand_world={xyz(result.hands_world["left"])} '
            f'right_hand_world={xyz(result.hands_world["right"])} '
            f'left_hand_local={xyz(result.hands_head_local["left"])} '
            f'right_hand_local={xyz(result.hands_head_local["right"])} '
            'input_symmetry_xyz='
            f'{np.round(result.input_symmetry_error, 3).tolist()} '
            'target_symmetry_xyz='
            f'{np.round(result.target_symmetry_error, 3).tolist()}'
        )

    def _publish_debug_pose(self, name, transform, frame_id):
        publisher = self._debug_pose_publishers.get(name)
        if publisher is None:
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        position = transform[:3, 3]
        orientation = matrix_quaternion(transform)
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(orientation[0])
        message.pose.orientation.y = float(orientation[1])
        message.pose.orientation.z = float(orientation[2])
        message.pose.orientation.w = float(orientation[3])
        publisher.publish(message)

    def _clamp_workspace(self, target, chain):
        maximum = chain.nominal_reach * self._workspace_reach_scale
        radius = float(np.linalg.norm(target[:3, 3]))
        if radius > maximum:
            target = target.copy()
            target[:3, 3] *= maximum / radius
            self._warn(
                f'{chain.tip_link}_workspace',
                f'Target outside {chain.tip_link} reach; clamped to {maximum:.3f} m',
            )
        return target

    def _report_ik_residual(self, runtime, result: IKResult):
        if result.converged:
            return
        self._warn(
            f'{runtime.side}_ik',
            f'{runtime.side} IK residual: position={result.position_error:.3f} m, '
            f'orientation={result.orientation_error:.3f} rad; '
            f'damping={result.damping:.4f}',
        )

    def _publish_hand_trajectories(self, now, dt):
        if self._joy_arrival is None or now - self._joy_arrival > self._joy_timeout_sec:
            self._warn('joy_stale', '/vr/joy is stale; holding hands')
            return
        left = self._left_closure_filter.update(
            self._left_closure_target, dt, self._finger_tau
        )
        right = self._right_closure_filter.update(
            self._right_closure_target, dt, self._finger_tau
        )
        self._publish_hand(self._left_hand, left, self._left_hand_publisher)
        self._publish_hand(self._right_hand, right, self._right_hand_publisher)

    def _publish_hand(self, config, closure, publisher):
        if not config['names']:
            return
        positions = config['open'] + closure * (config['closed'] - config['open'])
        debug_publisher = (
            self._debug_left_hand_publisher
            if config is self._left_hand
            else self._debug_right_hand_publisher
        )
        self._publish_hand_positions(
            config, positions, publisher, debug_publisher
        )
        self._hands_commanded = True

    def _publish_hand_positions(
        self, config, positions, publisher, debug_publisher
    ):
        message = JointTrajectory()
        message.joint_names = list(config['names'])
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = _duration(self._trajectory_time_sec)
        message.points = [point]
        debug_publisher.publish(message)
        if not self._dry_run:
            publisher.publish(message)

    def _publish_hold_trajectories(self):
        runtimes = [self._arms.get('left'), self._arms.get('right')]
        if all(runtime is not None and runtime.command is not None for runtime in runtimes):
            message = JointTrajectory()
            message.joint_names = [
                name for runtime in runtimes for name in runtime.chain.joint_names
            ]
            point = JointTrajectoryPoint()
            point.positions = [
                float(fallback if self._arm_range_probe is not None
                      else self._joint_positions.get(name, fallback))
                for runtime in runtimes
                for name, fallback in zip(runtime.chain.joint_names, runtime.command)
            ]
            point.time_from_start = _duration(
                min(0.05, self._trajectory_time_sec)
            )
            message.points = [point]
            self._publish_arm_command(message)

        if not self._hands_commanded:
            return
        hand_states = (
            (
                self._left_hand,
                self._left_closure_filter.value,
                self._left_hand_publisher,
                self._debug_left_hand_publisher,
            ),
            (
                self._right_hand,
                self._right_closure_filter.value,
                self._right_hand_publisher,
                self._debug_right_hand_publisher,
            ),
        )
        for config, closure, publisher, debug_publisher in hand_states:
            if not config['names']:
                continue
            fallback = config['open'] + closure * (
                config['closed'] - config['open']
            )
            positions = [
                self._joint_positions.get(name, float(value))
                for name, value in zip(config['names'], fallback)
            ]
            self._publish_hand_positions(
                config, positions, publisher, debug_publisher
            )

    def _is_enabled(self, now):
        return (
            self._teleop_active
            and self._active_arrival is not None
            and now - self._active_arrival <= self._active_timeout_sec
        )

    def _publish_zero_velocity(self):
        self._publish_velocity_command(Twist())
        self._velocity_streaming = False

    def _publish_velocity_command(self, message):
        self._debug_cmd_vel_publisher.publish(message)
        if not self._dry_run:
            self._cmd_vel_publisher.publish(message)

    def _warn(self, key, message):
        now = time.monotonic()
        if now - self._warning_times.get(key, 0.0) >= 2.0:
            self.get_logger().warning(message)
            self._warning_times[key] = now

    def stop(self):
        """Publish the process-local safe locomotion state."""
        self._publish_zero_velocity()
        self._publish_hold_trajectories()


def _optional_names(values: Sequence[str], sentinel: str):
    names = tuple(str(value) for value in values)
    if not names or names == (sentinel,):
        return None
    return names


def _clamp(value, magnitude):
    return max(-magnitude, min(magnitude, float(value)))


def _duration(seconds):
    nanoseconds = int(round(seconds * 1_000_000_000))
    return Duration(
        sec=nanoseconds // 1_000_000_000,
        nanosec=nanoseconds % 1_000_000_000,
    )


def main(args=None):
    rclpy.init(args=args)
    node = R1KinematicsControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            if rclpy.ok():
                node.stop()
        except BaseException:
            pass
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
