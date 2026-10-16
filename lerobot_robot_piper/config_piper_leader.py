from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("piper_leader")
@dataclass
class PiperLeaderConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str

    # python-can interface used to open `port`. The default keeps the Linux SocketCAN path
    # (`port` is an interface name such as "can0"). Without SocketCAN (macOS, Windows) the
    # gs_usb adapter shipped with the arm can be opened through libusb with "gs_usb"
    # (`port` is the adapter index, e.g. "0"), and serial CAN modules with "slcan"
    # (`port` is the serial device, e.g. "/dev/tty.usbmodem1101").
    can_interface: str = "socketcan"
    can_bitrate: int = 1_000_000

    # Sets the arm in torque mode with the gripper motor set to this value.
    gripper_open_pos: float = 50.0
