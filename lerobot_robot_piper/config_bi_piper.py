from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

from .config_piper import PiperFollowerArmConfig


@RobotConfig.register_subclass("bi_piper_follower")
@dataclass(kw_only=True)
class BiPiperFollowerConfig(RobotConfig):
    # Per-arm Piper follower configs. These map to
    # --robot.left_arm_config.port and --robot.right_arm_config.port.
    left_arm_config: PiperFollowerArmConfig
    right_arm_config: PiperFollowerArmConfig

    # Top-level cameras shared by the bimanual robot. Per-arm cameras declared
    # inside left/right_arm_config are prefixed in observations.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
