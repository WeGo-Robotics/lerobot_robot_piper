from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

from .config_piper_leader import PiperLeaderArmConfig


@TeleoperatorConfig.register_subclass("bi_piper_leader")
@dataclass(kw_only=True)
class BiPiperLeaderConfig(TeleoperatorConfig):
    # Per-arm Piper leader configs. These map to
    # --teleop.left_arm_config.port and --teleop.right_arm_config.port.
    left_arm_config: PiperLeaderArmConfig
    right_arm_config: PiperLeaderArmConfig

    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)
