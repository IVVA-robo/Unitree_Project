"""One-command entry point for the strictly video-only Robot POV service."""

import argparse
import asyncio
import logging
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
from typing import Optional, Sequence

from .config import PROFILES, SUPPORTED_SOURCES, PovConfig, load_config
from .frame_pipeline import FrameHub, FrameProcessor
from .server import RobotPovWebServer, WEBRTC_AVAILABLE
from .sources import (
    MockSource,
    OpenCVSource,
    ROS_AVAILABLE,
    RosStereoSource,
    UNITREE_SDK_AVAILABLE,
    UNITREE_SDK_IMPORT_ERROR,
    UnitreeVideoSource,
)


LOGGER = logging.getLogger('r1_robot_pov')


def _default_env_file(name: str) -> Optional[str]:
    source_path = Path(__file__).resolve().parents[1] / 'config' / name
    if source_path.is_file():
        return str(source_path)
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('r1_robot_pov')) / 'config' / name
        return str(installed) if installed.is_file() else None
    except (ImportError, LookupError):
        return None


class RosSourceRuntime:
    """Own one rclpy executor without exposing any control interfaces."""

    def __init__(self, config: PovConfig, hub: FrameHub, processor: FrameProcessor):
        if not ROS_AVAILABLE:
            raise RuntimeError('ROS camera source requires rclpy and sensor_msgs')
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=[])
        right_topic = (
            config.right_topic
            if config.layout in ('stereo', 'top-bottom')
            else None
        )
        self.node = RosStereoSource(
            hub,
            processor,
            left_topic=config.left_topic,
            right_topic=right_topic,
            sync_slop_s=config.stereo_max_skew_sec,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread: Optional[threading.Thread] = None

    def start(self):
        self.thread = threading.Thread(
            target=self.executor.spin,
            name='robot-pov-ros-camera',
            daemon=True,
        )
        self.thread.start()

    def stop(self, timeout=3.0):
        self.executor.shutdown(timeout_sec=max(0.0, float(timeout)))
        try:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        finally:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(max(0.0, float(timeout)))


class MdnsPublisher:
    """Publish an optional LAN-only alias and service through Avahi."""

    def __init__(self, config: PovConfig):
        self.config = config
        self.processes = []

    def start(self):
        name = self.config.mdns_name.rstrip('.')
        # avahi-publish -a expects the host label without the .local suffix;
        # passing "robot-pov.local" makes the address publisher exit while
        # the service publisher still appears to succeed.
        if name.lower().endswith('.local'):
            name = name[:-len('.local')]
        address = self.config.lan_ip
        executable = shutil.which('avahi-publish')
        if not executable or not name or address.startswith('127.'):
            return
        # Avahi publishes the laptop's own hostname automatically. Its CLI
        # rejects arbitrary .local aliases on this installation, so publish
        # only the service record; the hostname remains resolvable via Avahi.
        commands = [[
            executable, '-s', 'Robot POV', '_http._tcp',
            str(self.config.port), 'path=/', 'video-only=true',
        ]]
        for command in commands:
            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                self.processes.append(process)
            except OSError as exc:
                LOGGER.warning('mDNS publish unavailable: %s', exc)
                break

    def stop(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        self.processes.clear()


def _frame_processor(config: PovConfig) -> FrameProcessor:
    # Public dotenv crop order is top,right,bottom,left; the processor uses
    # left,top,right,bottom internally.
    top, right, bottom, left = config.crop
    processor_profile = config.profile
    if config.source == 'unitree' and config.layout == 'mono':
        profile = config.video_profile
        processor_profile = {
            'width': profile.width,
            'height': profile.height,
            'mono_width': round(profile.height * 16.0 / 9.0),
            'mono_height': profile.height,
            'fps': profile.fps,
            'jpeg_quality': profile.jpeg_quality,
            'video_bitrate': profile.video_bitrate_kbps * 1000,
        }
    return FrameProcessor(
        profile=processor_profile,
        swap_left_right=config.swap_eyes,
        crop_pixels=(left, top, right, bottom),
        rotation=config.rotation,
        flip_horizontal=config.flip_horizontal,
        flip_vertical=config.flip_vertical,
    )


def _source(config: PovConfig, hub: FrameHub, processor: FrameProcessor):
    profile = config.video_profile
    if config.source == 'mock':
        return MockSource(
            hub,
            processor,
            fps=profile.fps,
            stereo=config.layout in ('stereo', 'top-bottom'),
        )
    if config.source == 'ros':
        return RosSourceRuntime(config, hub, processor)
    if config.source == 'unitree':
        if not UNITREE_SDK_AVAILABLE:
            raise RuntimeError(
                'Unitree video source requires unitree_sdk2py; '
                f'import failed: {UNITREE_SDK_IMPORT_ERROR}'
            )
        return UnitreeVideoSource(
            hub,
            processor,
            network_interface=config.unitree_interface,
            rpc_timeout_s=config.unitree_timeout_sec,
            reconnect_delay_s=config.reconnect_delay_sec,
        )
    if config.source in ('usb', 'rtsp'):
        right = (
            config.right_source
            if config.layout in ('stereo', 'top-bottom')
            else None
        )
        return OpenCVSource(
            hub,
            config.left_source,
            right,
            processor,
            name=config.source,
            reconnect_delay_s=config.reconnect_delay_sec,
            capture_fps=profile.fps,
        )
    raise ValueError(f'unsupported source: {config.source}')


def _arguments(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('serve', 'mock', 'exhibition'))
    parser.add_argument('--env-file', default='')
    parser.add_argument('--source', choices=SUPPORTED_SOURCES)
    parser.add_argument('--profile', choices=tuple(PROFILES))
    parser.add_argument('--transport', choices=('auto', 'webrtc', 'mjpeg'))
    parser.add_argument('--layout', choices=('mono', 'stereo', 'top-bottom'))
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument('--no-mdns', action='store_true')
    parser.add_argument(
        '--no-discovery',
        action='store_true',
        help='Do not bind UDP 9091; the live VR bridge owns discovery',
    )
    arguments, unknown = parser.parse_known_args(argv)
    unexpected = [item for item in unknown if item != '--ros-args']
    # launch_ros appends remap/log arguments after --ros-args. They are not
    # consumed because this process intentionally creates no ROS control node.
    if unexpected and '--ros-args' not in unknown:
        parser.error(f'unrecognized arguments: {" ".join(unexpected)}')
    return arguments


def _configured(arguments) -> PovConfig:
    env_file = arguments.env_file or None
    if env_file is None:
        default_name = (
            'robot_pov.mock.env'
            if arguments.command == 'mock'
            else 'robot_pov.exhibition.env'
        )
        env_file = _default_env_file(default_name)
    config = load_config(env_file)
    overrides = {}
    if arguments.command == 'mock':
        overrides.update(mode='mock', source='mock')
    elif arguments.command == 'exhibition':
        overrides['mode'] = 'exhibition'
    for argument_name, field_name in (
        ('source', 'source'),
        ('profile', 'profile'),
        ('transport', 'transport'),
        ('layout', 'layout'),
        ('host', 'bind_host'),
        ('port', 'port'),
    ):
        value = getattr(arguments, argument_name)
        if value is not None:
            overrides[field_name] = value
    if arguments.no_discovery:
        overrides['discovery_enabled'] = False
    return config.with_overrides(**overrides)


async def _run(config: PovConfig, duration: float, enable_mdns: bool):
    hub = FrameHub(stale_after=config.stale_after_sec)
    processor = _frame_processor(config)
    source = _source(config, hub, processor)
    server = RobotPovWebServer(config, hub)
    mdns = MdnsPublisher(config)
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(signum, stop_event.set)
        except NotImplementedError:  # pragma: no cover - Windows fallback.
            pass

    source.start()
    try:
        await server.start()
        if enable_mdns:
            mdns.start()
        print('Robot POV is running (strict video-only mode).')
        print(f'Numeric LAN URL: {config.numeric_url}')
        if config.mdns_url:
            print(f'mDNS URL:       {config.mdns_url}')
        print(f'QR image:       {config.numeric_url}qr.png')
        print(
            f'Source={config.source}, profile={config.profile}, '
            f'transport={config.transport}, WebRTC={WEBRTC_AVAILABLE}'
        )
        if duration > 0.0:
            try:
                await asyncio.wait_for(stop_event.wait(), timeout=duration)
            except asyncio.TimeoutError:
                pass
        else:
            await stop_event.wait()
    finally:
        mdns.stop()
        await server.close()
        source.stop()


def main(argv: Optional[Sequence[str]] = None):
    """Run viewer or delegate to the non-mutating preflight command."""
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments and arguments[0] == 'preflight':
        from .preflight import main as preflight_main

        return preflight_main(arguments[1:])
    parsed = _arguments(arguments)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    try:
        config = _configured(parsed)
        asyncio.run(_run(config, parsed.duration, not parsed.no_mdns))
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        LOGGER.error('%s', exc)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
