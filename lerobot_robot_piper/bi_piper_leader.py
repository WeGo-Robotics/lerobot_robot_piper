from __future__ import annotations

import logging
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_bi_piper_leader import BiPiperLeaderConfig
from .config_piper_leader import PiperLeaderConfig
from .piper_leader import PiperLeader

logger = logging.getLogger(__name__)


class BiPiperLeader(Teleoperator):
    """Bimanual Piper leader teleoperator composed of two PiperLeader arms."""

    config_class = BiPiperLeaderConfig
    name = "bi_piper_leader"

    def __init__(self, config: BiPiperLeaderConfig):
        super().__init__(config)
        self.config = config
        self.id = config.id

        left_arm_config = PiperLeaderConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            gripper_open_pos=config.left_arm_config.gripper_open_pos,
        )
        right_arm_config = PiperLeaderConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            gripper_open_pos=config.right_arm_config.gripper_open_pos,
        )

        self.left_arm = PiperLeader(left_arm_config)
        self.right_arm = PiperLeader(right_arm_config)

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    @staticmethod
    def _prefix(side: str, values: dict[str, Any]) -> dict[str, Any]:
        return {f"{side}_{key}": value for key, value in values.items()}

    @cached_property
    def action_features(self) -> dict:
        return {
            **self._prefix("left", self.left_arm.action_features),
            **self._prefix("right", self.right_arm.action_features),
        }

    @cached_property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        try:
            self.left_arm.connect(calibrate=calibrate)
            self.right_arm.connect(calibrate=calibrate)
        except Exception:
            logger.exception(f"{self} failed to connect.")
            self.disconnect()
            raise

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def _load_calibration(self, fpath: Path | None = None) -> None:
        pass

    def _save_calibration(self, fpath: Path | None = None) -> None:
        pass

    def configure(self) -> None:
        pass

    def setup_motors(self) -> None:
        self.left_arm.setup_motors()
        self.right_arm.setup_motors()

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        return {
            **self._prefix("left", self.left_arm.get_action()),
            **self._prefix("right", self.right_arm.get_action()),
        }

    def is_protected(self) -> bool:
        return self.left_arm.is_protected() or self.right_arm.is_protected()

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        for arm in (self.right_arm, self.left_arm):
            try:
                arm.disconnect()
            except Exception:
                logger.exception(f"{self} failed to disconnect {arm}.")
