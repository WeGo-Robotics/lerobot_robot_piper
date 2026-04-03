import logging
import time
from typing import Any

from piper_sdk import C_PiperInterface_V2
from wego_piper.port_handler import PortHandler

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.motors_bus import MotorsBusBase

from .tables import (
    INITIALIZE_POSITION,
    MODEL_RESOLUTION_TABLE,
)

logger = logging.getLogger(__name__)


class PiperMotorsBus(MotorsBusBase):

    apply_drive_mode = False

    def __init__(
        self,
        id: str,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
    ):
        super().__init__(port, motors, calibration)

        self.port_handler = PortHandler()
        self.id = id
        self._is_connected = False
        self.piper = C_PiperInterface_V2(port)
        logger.info(f"{id} : {port} is selected.")

    # ---- MotorsBusBase abstract implementations ----

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, handshake: bool = True) -> None:
        self.port_handler.setupPort(self.piper)
        if not self.port_handler.openPort():
            raise ConnectionError(f"Failed to open port for {self.id}")
        self._is_connected = True

    def disconnect(self, disable_torque: bool = True) -> None:
        if disable_torque:
            self.parking()
            self.piper.DisablePiper()
        self.port_handler.closePort()
        self._is_connected = False

    def read(self, data_name: str, motor: str) -> int | float:
        pos = self.get_action()
        return pos.get(motor, 0)

    def write(self, data_name: str, motor: str, value: int | float) -> None:
        current = self.get_action()
        current[motor] = value
        self.set_action(current, is_conv=True)

    def sync_read(self, data_name: str, motors: str | list[str] | None = None) -> dict[str, int | float]:
        pos = self.get_action()
        if motors is None:
            return pos
        if isinstance(motors, str):
            motors = [motors]
        return {m: pos[m] for m in motors if m in pos}

    def sync_write(self, data_name: str, values: dict[str, int | float]) -> None:
        self.set_action(values, is_conv=True)

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        retry = 50  # 5 seconds max
        while not self.piper.EnablePiper() and retry:
            retry -= 1
            time.sleep(0.1)
        logger.info(f"{self.piper.GetArmEnableStatus()}")
        if not retry:
            logger.warning(f"{self.id} enable_torque timed out, continuing anyway")
        else:
            logger.info(f"{self.id} torque on.")

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        self.piper.DisablePiper()

    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.calibration

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        self.calibration = calibration_dict

    # ---- Piper-specific methods ----

    @property
    def is_calibrated(self) -> bool:
        return True

    def clear_gripper(self):
        self.piper.GripperCtrl(0, 1000, 0x03, 0)

    def parking(self):
        timeout = 100  # 10sec
        self.set_action(INITIALIZE_POSITION, False)
        time.sleep(0.1)
        status = self.piper.GetArmStatus()

        while status.arm_status.motion_status and timeout:
            self.set_action(INITIALIZE_POSITION, False)
            time.sleep(0.1)
            status = self.piper.GetArmStatus()
            timeout -= 1

    def set_slave(self):
        self.piper.MasterSlaveConfig(0xFC, 0, 0, 0)

    def set_master(self):
        self.piper.MasterSlaveConfig(0xFA, 0, 0, 0)

    def get_action(self) -> dict[str, Any]:
        msg_joint = self.piper.GetArmJointMsgs()
        msg_gripr = self.piper.GetArmGripperMsgs()
        rlt = {
            "joint1": float(msg_joint.joint_state.joint_1),
            "joint2": float(msg_joint.joint_state.joint_2),
            "joint3": float(msg_joint.joint_state.joint_3),
            "joint4": float(msg_joint.joint_state.joint_4),
            "joint5": float(msg_joint.joint_state.joint_5),
            "joint6": float(msg_joint.joint_state.joint_6),
            "gripper": float(msg_gripr.gripper_state.grippers_angle),
        }
        return self._normalize(rlt)

    def get_control(self) -> dict[str, Any]:
        msg_joint = self.piper.GetArmJointCtrl()
        msg_gripr = self.piper.GetArmGripperCtrl()
        rlt = {
            "joint1": float(msg_joint.joint_ctrl.joint_1),
            "joint2": float(msg_joint.joint_ctrl.joint_2),
            "joint3": float(msg_joint.joint_ctrl.joint_3),
            "joint4": float(msg_joint.joint_ctrl.joint_4),
            "joint5": float(msg_joint.joint_ctrl.joint_5),
            "joint6": float(msg_joint.joint_ctrl.joint_6),
            "gripper": float(msg_gripr.gripper_ctrl.grippers_angle),
        }
        return self._normalize(rlt)

    def set_action(self, action: dict[str, Any], is_conv: bool = True) -> dict[str, Any]:
        if is_conv:
            action_denormalized = self._unnormalize(action)
        else:
            action_denormalized = action

        self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper.JointCtrl(
            int(action_denormalized["joint1"]),
            int(action_denormalized["joint2"]),
            int(action_denormalized["joint3"]),
            int(action_denormalized["joint4"]),
            int(action_denormalized["joint5"]),
            int(action_denormalized["joint6"]),
        )
        self.piper.GripperCtrl(abs(int(action_denormalized["gripper"])), 1000, 0x03, 0)
        return self.get_control()

    # ---- Normalization ----

    def _normalize(self, ids_values: dict[str, int]) -> dict[str, float]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        normalized_values = {}
        for motor, val in ids_values.items():
            min_ = self.calibration[motor].range_min
            max_ = self.calibration[motor].range_max
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode
            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            bounded_val = min(max_, max(min_, val))
            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
                normalized_values[motor] = -norm if drive_mode else norm
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                norm = ((bounded_val - min_) / (max_ - min_)) * 100
                normalized_values[motor] = 100 - norm if drive_mode else norm
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = MODEL_RESOLUTION_TABLE[self.motors[motor].model] - 1
                normalized_values[motor] = (val - mid) * 360 / max_res
            else:
                raise NotImplementedError

        return normalized_values

    def _unnormalize(self, ids_values: dict[str, float]) -> dict[str, int]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        unnormalized_values = {}
        for motor, val in ids_values.items():
            min_ = self.calibration[motor].range_min
            max_ = self.calibration[motor].range_max
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode
            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                val = -val if drive_mode else val
                bounded_val = min(100.0, max(-100.0, val))
                unnormalized_values[motor] = int(((bounded_val + 100) / 200) * (max_ - min_) + min_)
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                val = 100 - val if drive_mode else val
                bounded_val = min(100.0, max(0.0, val))
                unnormalized_values[motor] = int((bounded_val / 100) * (max_ - min_) + min_)
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = MODEL_RESOLUTION_TABLE[self.motors[motor].model] - 1
                unnormalized_values[motor] = int((val * max_res / 360) + mid)
            else:
                raise NotImplementedError

        return unnormalized_values
