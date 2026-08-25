# Discovery shim.
#
# LeRobot's register_third_party_plugins() imports the module named after the
# installed DISTRIBUTION (lerobot_robot_wego_piper). The real code lives in
# the lerobot_robot_piper package; importing it here runs the
# @register_subclass decorators that register piper_follower / piper_leader.
# Existing `import lerobot_robot_piper` code keeps working unchanged.

from lerobot_robot_piper import (
    PiperFollower,
    PiperFollowerConfig,
    PiperLeader,
    PiperLeaderConfig,
)

__all__ = ["PiperFollower", "PiperFollowerConfig", "PiperLeader", "PiperLeaderConfig"]
