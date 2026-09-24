from __future__ import annotations

import logging
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.cameras import Camera
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.robots import Robot

from .config_bi_piper import BiPiperFollowerConfig
from .config_piper import PiperFollowerConfig
from .piper_follower import PiperFollower

logger = logging.getLogger(__name__)


class BiPiperFollower(Robot):
    """Bimanual Piper follower robot composed of two PiperFollower arms."""

    config_class = BiPiperFollowerConfig
    name = "bi_piper_follower"

    def __init__(self, config: BiPiperFollowerConfig):
        super().__init__(config)
        self.config = config
        self.id = config.id
        self._top_level_cam_keys = set(config.cameras)

        camera_collisions = (
            self._top_level_cam_keys & set(config.left_arm_config.cameras)
            | self._top_level_cam_keys & set(config.right_arm_config.cameras)
        )
        if camera_collisions:
            raise ValueError(
                f"Top-level camera names collide with per-arm camera names: {sorted(camera_collisions)}"
            )

        left_arm_config = PiperFollowerConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            disable_torque_on_disconnect=config.left_arm_config.disable_torque_on_disconnect,
            cameras={**config.left_arm_config.cameras, **config.cameras},
            max_relative_target=config.left_arm_config.max_relative_target,
        )
        right_arm_config = PiperFollowerConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            disable_torque_on_disconnect=config.right_arm_config.disable_torque_on_disconnect,
            cameras=config.right_arm_config.cameras,
            max_relative_target=config.right_arm_config.max_relative_target,
        )

        self.left_arm = PiperFollower(left_arm_config)
        self.right_arm = PiperFollower(right_arm_config)
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

    def __str__(self) -> str:
        return f"{self.id} {self.__class__.__name__}"

    @staticmethod
    def _prefix(side: str, values: dict[str, Any]) -> dict[str, Any]:
        return {f"{side}_{key}": value for key, value in values.items()}

    @staticmethod
    def _unprefix(side: str, values: dict[str, Any]) -> dict[str, Any]:
        prefix = f"{side}_"
        return {key.removeprefix(prefix): value for key, value in values.items() if key.startswith(prefix)}

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            **self._prefix("left", self.left_arm._motors_ft),
            **self._prefix("right", self.right_arm._motors_ft),
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        out: dict[str, tuple] = {}
        for key, value in self.left_arm._cameras_ft.items():
            out[key if key in self._top_level_cam_keys else f"left_{key}"] = value
        for key, value in self.right_arm._cameras_ft.items():
            out[f"right_{key}"] = value
        return out

    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def get_cameras(self) -> dict[str, Camera]:
        return {
            **{
                key if key in self._top_level_cam_keys else f"left_{key}": camera
                for key, camera in self.left_arm.get_cameras().items()
            },
            **{f"right_{key}": camera for key, camera in self.right_arm.get_cameras().items()},
        }

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

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict: dict[str, Any] = {}
        for key, value in self.left_arm.get_observation().items():
            obs_dict[key if key in self._top_level_cam_keys else f"left_{key}"] = value
        for key, value in self.right_arm.get_observation().items():
            obs_dict[f"right_{key}"] = value
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        left_action = self._unprefix("left", action)
        right_action = self._unprefix("right", action)

        sent_left = self.left_arm.send_action(left_action) if left_action else {}
        sent_right = self.right_arm.send_action(right_action) if right_action else {}

        return {
            **self._prefix("left", sent_left),
            **self._prefix("right", sent_right),
        }

    def parking(self) -> None:
        self.left_arm.parking()
        self.right_arm.parking()

    def disconnect(self, disable_torque: bool = False) -> None:
        for arm in (self.right_arm, self.left_arm):
            try:
                arm.disconnect(disable_torque)
            except Exception:
                logger.exception(f"{self} failed to disconnect {arm}.")
