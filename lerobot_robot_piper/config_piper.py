from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("piper_follower")
@dataclass(kw_only=True)
class PiperFollowerConfig(RobotConfig):
    # Port to connect to the arm
    port: str

    # python-can interface used to open `port`. The default keeps the Linux SocketCAN path
    # (`port` is an interface name such as "can0"). Without SocketCAN (macOS, Windows) the
    # gs_usb adapter shipped with the arm can be opened through libusb with "gs_usb"
    # (`port` is the adapter index, e.g. "0"), and serial CAN modules with "slcan"
    # (`port` is the serial device, e.g. "/dev/tty.usbmodem1101").
    can_interface: str = "socketcan"
    can_bitrate: int = 1_000_000

    disable_torque_on_disconnect: bool = True

    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
