"""Offline subprocess behavior; none of these tests opens the robot network."""

import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/r1-loco-state-watch"
loader = importlib.machinery.SourceFileLoader("loco_state_watch", str(SCRIPT))
spec = importlib.util.spec_from_loader(loader.name, loader)
watch = importlib.util.module_from_spec(spec)
loader.exec_module(watch)


def options(*extra):
    return watch.arguments(["--interface", "test0", "--duration-sec", "0.1",
                            "--interval-sec", "0.5", *extra])


def fake_probe(tmp_path, scenarios):
    events = tmp_path / "events.jsonl"
    responses = tmp_path / "responses.json"
    responses.write_text(json.dumps(scenarios))
    probe = tmp_path / "probe"
    probe.write_text(
        f"#!{sys.executable}\n"
        "import json, pathlib, sys, time\n"
        f"events = pathlib.Path({str(events)!r})\n"
        f"responses = json.loads(pathlib.Path({str(responses)!r}).read_text())\n"
        "index = len(events.read_text().splitlines()) if events.exists() else 0\n"
        "with events.open('a') as stream:\n"
        "    stream.write(json.dumps({'args': sys.argv[1:], 'at': time.monotonic()}) + '\\n')\n"
        "response = responses[min(index, len(responses)-1)]\n"
        "time.sleep(response.get('sleep', 0))\n"
        "print(response.get('output', ''), flush=True)\n"
        "raise SystemExit(response.get('exit', 0))\n"
    )
    probe.chmod(0o755)
    return probe, events


def line(fsm=811, status="OK", code=0):
    return f"R1_LOCO_FSM_ID status={status} api=7001 code={code} fsm_id={fsm}"


@pytest.mark.parametrize("flag,value", [
    ("--interval-sec", "0.49"), ("--duration-sec", "120.1"),
    ("--duration-sec", "nan"), ("--interval-sec", "inf"),
    ("--probe-timeout-sec", "11"), ("--timeout-sec", "0"),
    ("--max-sample-age-sec", "0"), ("--interface", "test;echo"),
])
def test_invalid_bounds_rejected_before_probe(flag, value):
    with pytest.raises(SystemExit) as failure:
        options(flag, value)
    assert failure.value.code == 2


def test_duration_and_interval_must_be_explicit():
    with pytest.raises(SystemExit):
        watch.arguments(["--interface", "test0"])


def test_capture_changes_fsm_then_clears_failure_without_commands(tmp_path):
    probe, events = fake_probe(tmp_path, [
        {"output": line(811)}, {"output": line(816)},
        {"output": line(-1, "RPC_ERROR", 3102), "exit": 10},
    ])
    records = []
    result = watch.capture(options("--duration-sec", "1.25"), probe=probe, emit=records.append)
    samples = [item for item in records if item["event"] == "fsm_sample"]
    assert result == 1
    assert [item["fsm_id"] for item in samples] == [811, 816, None]
    assert samples[-1]["status"] == "rpc_error"
    assert samples[-1]["fresh"] is False
    assert samples[-1]["rpc_code"] == 3102
    assert all(item["timestamp"].endswith("+00:00") for item in records)
    assert all(item["fsm_id"] is None for item in records if item["event"] != "fsm_sample")
    invocations = [json.loads(item) for item in events.read_text().splitlines()]
    assert len(invocations) == 3
    assert all(item["args"] == ["--interface", "test0", "--timeout-sec", "0.5"]
               for item in invocations)
    # The request starts are rate-limited; child interpreter startup can vary
    # with system load and must not make this assertion flaky.
    starts = [item for item in records if item['event'] == 'query_started']
    assert all(later['elapsed_sec'] - earlier['elapsed_sec'] >= 0.49
               for earlier, later in zip(starts, starts[1:]))
    assert records[-1]["event"] == "watch_complete"
    assert records[-1]["fresh"] is False


def test_slow_success_is_stale_and_cannot_retain_fsm(tmp_path):
    probe, _ = fake_probe(tmp_path, [{"output": line(), "sleep": 0.15}])
    records = []
    assert watch.capture(options("--duration-sec", "0.35", "--max-sample-age-sec", "0.1"),
                         probe=probe, emit=records.append) == 1
    sample = next(item for item in records if item["event"] == "fsm_sample")
    assert sample["status"] == "stale"
    assert sample["fsm_id"] is None
    assert sample["fresh"] is False
    assert sample["sample_age_upper_bound_sec"] >= 0.15


def test_process_timeout_is_capped_by_capture_duration(tmp_path):
    probe, events = fake_probe(tmp_path, [{"sleep": 5, "output": line()}])
    records = []
    start = time.monotonic()
    assert watch.capture(options("--duration-sec", "0.2", "--probe-timeout-sec", "4"),
                         probe=probe, emit=records.append) == 1
    assert time.monotonic() - start < 1
    sample = next(item for item in records if item["event"] == "fsm_sample")
    assert sample["status"] == "timeout"
    assert sample["fsm_id"] is None
    assert len(events.read_text().splitlines()) == 1


@pytest.mark.parametrize("output,exit_code,status", [
    (line(), 1, "probe_failed"), (line() + "\n" + line(816), 0, "invalid_output"),
    ("nothing", 0, "invalid_output"), (line(-1), 0, "invalid_output"),
    (line(811, "OK", 127), 0, "rpc_error"),
])
def test_ambiguous_or_failed_results_never_authorize_a_state(output, exit_code, status):
    result = watch.sample_result(exit_code, output, 0.02, 1)
    assert result["status"] == status
    assert result["fsm_id"] is None
    assert result["fresh"] is False


def test_missing_probe_clears_state(tmp_path):
    records = []
    assert watch.capture(options(), probe=tmp_path / "missing", emit=records.append) == 1
    assert records[1]["status"] == "unavailable"
    assert records[1]["fsm_id"] is None
    assert records[-1]["fresh"] is False
