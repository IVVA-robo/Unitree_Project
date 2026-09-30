"""Verify the actual mock-smoke parser against ROS message-loss diagnostics."""

from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.parametrize('positions,expected', [('0.0, 0.0', 0), ('0.1, 0.0', 1)])
def test_head_zero_check_reads_positions_not_warning_counts(tmp_path, positions, expected):
    script = (Path(__file__).resolve().parents[1] / 'scripts/r1-live-writer-mock-smoke')
    function = script.read_text().split('assert_head_points() {', 1)[1]
    parser = function.split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
    capture = tmp_path / 'head.txt'
    capture.write_text(
        'A message was lost!!!\n total count change:1\n total count: 1---\n'
        '[trajectory_msgs.msg.JointTrajectoryPoint(positions=[' + positions + '], '
        'velocities=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=0))]\n'
    )
    result = subprocess.run([sys.executable, '-c', parser, str(capture), 'zero', 'test'],
                            capture_output=True, text=True)
    assert result.returncode == expected, result.stderr
