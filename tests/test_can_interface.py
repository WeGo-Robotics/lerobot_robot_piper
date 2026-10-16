"""`can_interface` option of PiperMotorsBus — runs without an arm or an adapter.

The non-SocketCAN path is exercised end to end on python-can's `virtual` interface,
with a thread standing in for the arm: a follower sends status feedback (0x2A1), a leader
in teach mode sends joint control frames (0x155).
"""

import threading
import time
import uuid

import can
import pytest

from lerobot_robot_piper.motors import piper_motors_bus
from lerobot_robot_piper.motors.piper_motors_bus import PiperMotorsBus

# What a powered arm keeps sending. The SDK reports a joint rate only when all three
# frames of the group arrive, as they do from a real arm.
FOLLOWER_STATUS = (0x2A1,)  # arm status feedback
FOLLOWER_JOINTS = (0x2A5, 0x2A6, 0x2A7)  # joint feedback
LEADER_CTRL = (0x155, 0x156, 0x157)  # leader in teach mode: joint control
FIRMWARE = (0x4AF,)  # answer to a firmware query — what a transmit echo looks like here


def _bus(port, **kw):
    return PiperMotorsBus(id="test", port=port, motors={}, **kw)


class _Recorder:
    """Stands in for C_PiperInterface_V2 and records how it was built."""

    calls: list = []

    def __init__(self, *args, **kwargs):
        _Recorder.calls.append(("init", args, kwargs))

    def CreateCanBus(self, *args, **kwargs):  # noqa: N802
        _Recorder.calls.append(("CreateCanBus", args, kwargs))


@pytest.fixture
def recorder(monkeypatch):
    _Recorder.calls = []
    monkeypatch.setattr(piper_motors_bus, "C_PiperInterface_V2", _Recorder)
    return _Recorder.calls


def test_socketcan_default_builds_the_interface_exactly_as_before(recorder):
    _bus("can0")
    assert recorder == [("init", ("can0",), {})]


def test_gs_usb_passes_the_adapter_index_as_an_int(recorder):
    _bus("0", can_interface="gs_usb")
    (_, init_args, init_kw), (_, args, kw) = recorder
    # the SDK caches one instance per name — keep backends apart
    assert init_args == ("gs_usb:0",)
    assert init_kw == {"judge_flag": False, "can_auto_init": False}
    assert args == (0,)
    assert kw == {"bustype": "gs_usb", "expected_bitrate": 1_000_000, "judge_flag": False}


def test_slcan_keeps_the_serial_device_as_is(recorder):
    _bus("/dev/tty.usbmodem1101", can_interface="slcan", can_bitrate=500_000)
    _, args, kw = recorder[1]
    assert args == ("/dev/tty.usbmodem1101",)
    assert kw["bustype"] == "slcan" and kw["expected_bitrate"] == 500_000


@pytest.fixture
def channel():
    return f"piper-test-{uuid.uuid4().hex[:8]}"


def _fake_arm(channel, frame_ids, stop):
    with can.Bus(interface="virtual", channel=channel) as arm:
        while not stop.is_set():
            for frame_id in frame_ids:
                arm.send(can.Message(arbitration_id=frame_id, data=[0] * 8, is_extended_id=False))
            time.sleep(0.005)


@pytest.mark.parametrize(
    "frame_ids",
    [FOLLOWER_STATUS, FOLLOWER_JOINTS, LEADER_CTRL],
    ids=["follower-status", "follower-joints", "leader"],
)
def test_virtual_bus_connects_when_the_arm_talks(channel, frame_ids):
    stop = threading.Event()
    t = threading.Thread(target=_fake_arm, args=(channel, frame_ids, stop), daemon=True)
    t.start()
    try:
        bus = _bus(channel, can_interface="virtual")
        bus.connect()
        assert bus.is_connected
        bus.disconnect(disable_torque=False)
    finally:
        stop.set()
        t.join(timeout=1)


def test_virtual_bus_without_an_arm_fails_loudly(channel):
    bus = _bus(channel, can_interface="virtual")
    with pytest.raises(ConnectionError, match="No frame from the arm"):
        bus.connect()
    assert not bus.is_connected


def test_frames_that_are_not_from_the_arm_do_not_count(channel):
    """gs_usb hands transmit echoes back as frames; those must not pass as an arm."""
    stop = threading.Event()
    t = threading.Thread(target=_fake_arm, args=(channel, FIRMWARE, stop), daemon=True)
    t.start()
    try:
        bus = _bus(channel, can_interface="virtual")
        with pytest.raises(ConnectionError, match="No frame from the arm"):
            bus.connect()
    finally:
        stop.set()
        t.join(timeout=1)


def test_a_reconnect_needs_fresh_frames(channel):
    """The SDK keeps one instance per port, with the last frames in it. A reconnect after
    the arm went quiet must not pass on what the previous connection received."""
    stop = threading.Event()
    t = threading.Thread(target=_fake_arm, args=(channel, FOLLOWER_STATUS, stop), daemon=True)
    t.start()
    bus = _bus(channel, can_interface="virtual")
    bus.connect()
    bus.disconnect(disable_torque=False)
    stop.set()
    t.join(timeout=1)

    with pytest.raises(ConnectionError, match="No frame from the arm"):
        _bus(channel, can_interface="virtual").connect()
