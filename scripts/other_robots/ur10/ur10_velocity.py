"""
Backward-compatible wrapper for the UR10 Giskard entry point.

The canonical implementation now lives in
``giskardpy_ros.configs.other_robots.ur10`` (config classes) and
``scripts.other_robots.ur10.ur10_velocity`` (entry point).

This script re-exports the config classes so that existing imports
continue to work, and delegates ``python ur10_velocity.py <xacro>``
to the original argparse-based workflow for users who invoke the
script directly.
"""
from __future__ import annotations

import argparse
import os

from giskardpy.model.collision_world_syncer import CollisionCheckerLib
from giskardpy.qp.qp_controller_config import QPControllerConfig
from giskardpy_ros.configs.behavior_tree_config import (
    ClosedLoopBTConfig,
    StandAloneBTConfig,
)
from giskardpy_ros.configs.giskard import Giskard
from giskardpy_ros.configs.other_robots.ur10 import (
    UR10StandAloneRobotInterfaceConfig,
    UR10VelocityInterface,
    WorldWithUR10Config,
)
from giskardpy_ros.ros2 import rospy
from giskardpy_ros.ros2.visualization_mode import VisualizationMode
from giskardpy_ros.utils.utils import load_xacro

__all__ = [
    "WorldWithUR10Config",
    "UR10VelocityInterface",
    "UR10StandAloneRobotInterfaceConfig",
]


def main(args: argparse.Namespace) -> None:
    """
    Start the Giskard motion planner using an explicit xacro path.

    This preserves the original CLI interface for direct invocation::

        python ur10_velocity.py models/ur10_femto_bolt.urdf.xacro
        python ur10_velocity.py models/ur10_femto_bolt.urdf.xacro --standalone

    :param args: Parsed arguments containing ``robot_description``
        and ``standalone``.
    """
    rospy.init_node("giskard")
    xacro_path = os.path.abspath(args.robot_description)
    robot_description = load_xacro(xacro_path)

    if args.standalone:
        behavior_tree_config = StandAloneBTConfig(
            visualization_mode=VisualizationMode.VisualsFrameLocked
        )
        robot_interface_config = UR10StandAloneRobotInterfaceConfig()
    else:
        behavior_tree_config = ClosedLoopBTConfig(
            visualization_mode=VisualizationMode.VisualsFrameLocked
        )
        robot_interface_config = UR10VelocityInterface()

    giskard = Giskard(
        world_config=WorldWithUR10Config(urdf=robot_description),
        collision_checker_id=CollisionCheckerLib.none,
        robot_interface_config=robot_interface_config,
        behavior_tree_config=behavior_tree_config,
        qp_controller_config=QPControllerConfig(
            target_frequency=80, prediction_horizon=35
        ),
    )
    giskard.live()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Start the Giskard motion planner for a UR10 robot."
    )
    parser.add_argument(
        "robot_description",
        help="Path to the robot description xacro file.",
    )
    parser.add_argument(
        "--standalone",
        action="store_true",
        help="Run in standalone mode (simulated robot, publishes TF).",
    )
    arguments = parser.parse_args()
    main(arguments)
