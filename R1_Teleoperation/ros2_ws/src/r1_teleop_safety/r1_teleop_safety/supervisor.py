"""ROS safety-kill latch used by dry-run head and locomotion controllers."""

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .environment import ActuationPolicy


class R1SafetySupervisor(Node):
    """Publish a fail-closed emergency-kill state and audited status."""

    def __init__(self):
        super().__init__('r1_safety_supervisor')
        self.declare_parameter('kill_topic', '/r1/safety/kill')
        self.declare_parameter(
            'kill_request_topic', '/r1/safety/kill_request'
        )
        self.declare_parameter('status_topic', '/r1/safety/status')
        self.declare_parameter('allow_dry_run_release', True)
        # The physical writer requires a fresh kill heartbeat within 0.5 s.
        # Twenty hertz leaves enough scheduling margin during SDK discovery,
        # diagnostics and live status sampling while remaining lightweight.
        self.declare_parameter('publish_rate_hz', 20.0)

        self._policy = ActuationPolicy.from_environment()
        self._kill_topic = str(self.get_parameter('kill_topic').value)
        self._kill_request_topic = str(
            self.get_parameter('kill_request_topic').value
        )
        self._status_topic = str(self.get_parameter('status_topic').value)
        self._allow_dry_run_release = bool(
            self.get_parameter('allow_dry_run_release').value
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not all(
            topic.startswith('/')
            for topic in (
                self._kill_topic,
                self._kill_request_topic,
                self._status_topic,
            )
        ):
            raise ValueError('safety topics must be absolute ROS topic names')
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._kill_publisher = self.create_publisher(Bool, self._kill_topic, latch_qos)
        self._status_publisher = self.create_publisher(
            String, self._status_topic, latch_qos
        )
        self._kill_request_subscription = self.create_subscription(
            Bool,
            self._kill_request_topic,
            self._kill_request,
            latch_qos,
        )
        self.create_service(SetBool, '/r1/safety/set_kill', self._set_kill)
        self.create_service(Trigger, '/r1/safety/emergency_stop', self._emergency_stop)
        self.create_service(Trigger, '/r1/safety/get_status', self._status)

        # A fresh process is always killed until an operator explicitly releases
        # it. This is a software interlock, not a replacement for physical E-stop.
        self._killed = True
        self._reason = 'startup_fail_closed'
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self._publish()
        self.get_logger().warning(
            'Safety kill is asserted on startup. Release only for dry-run or '
            'after every live interlock is confirmed.'
        )

    def _can_release(self):
        if self._policy.dry_run:
            return self._allow_dry_run_release, 'dry-run release allowed'
        if self._policy.live_authorized:
            return True, 'all live environment interlocks are set'
        required = ', '.join(self._policy.missing_live_interlocks())
        return False, f'live release blocked; missing {required}'

    def _set_kill(self, request, response):
        if request.data:
            self._killed = True
            self._reason = 'operator_kill'
            response.success = True
            response.message = 'kill asserted'
        else:
            allowed, reason = self._can_release()
            if allowed:
                self._killed = False
                self._reason = 'operator_release'
                response.success = True
                response.message = reason
            else:
                self._killed = True
                self._reason = 'release_rejected'
                response.success = False
                response.message = reason
        self._publish()
        return response

    def _emergency_stop(self, _request, response):
        self._killed = True
        self._reason = 'emergency_stop_service'
        self._publish()
        response.success = True
        response.message = 'kill asserted; downstream controllers must stop/hold'
        return response

    def _kill_request(self, message):
        """Latch a downstream writer fault; false can never clear the latch."""
        if message is None or not bool(message.data):
            return
        self._killed = True
        self._reason = 'downstream_kill_request'
        self._publish()

    def _status(self, _request, response):
        response.success = True
        response.message = self._status_text()
        return response

    def _status_text(self):
        return (
            f'{self._policy.summary()} kill={str(self._killed).lower()} '
            f'reason={self._reason} physical_writer=external_fail_closed'
        )

    def _publish(self):
        self._kill_publisher.publish(Bool(data=self._killed))
        self._status_publisher.publish(String(data=self._status_text()))


def main(args=None):
    """Run the safety-kill latch until interrupted."""
    rclpy.init(args=args)
    node = R1SafetySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._killed = True
            node._reason = 'shutdown'
            node._publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
