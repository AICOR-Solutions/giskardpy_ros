"""
Start the Giskard motion planner for a UR10 robot.

Reads ``robot_description`` from a ROS parameter. Falls back to the
default POC-thyssen xacro when the parameter is not set.

The ``mode`` parameter controls hardware vs. simulation:

- ``"hardware"``: :class:`ClosedLoopBTConfig` + :class:`UR10VelocityInterface`
- ``"standalone"`` (default): :class:`StandAloneBTConfig` +
  :class:`UR10StandAloneRobotInterfaceConfig`
"""
from __future__ import annotations

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
from rclpy import Parameter
from rclpy.exceptions import ParameterUninitializedException


def main() -> None:
    """
    Entry point for the UR10 Giskard node.

    Declares ROS parameters ``robot_description`` and ``mode``, builds the
    appropriate configuration, and starts the Giskard behaviour-tree loop.
    """
    rospy.init_node("giskard")
    rospy.node.declare_parameters(
        namespace="",
        parameters=[
            ("robot_description", Parameter.Type.STRING),
            ("mode", Parameter.Type.STRING),
        ],
    )

    try:
        robot_description = rospy.node.get_parameter("robot_description").value
    except ParameterUninitializedException:
        robot_description = load_xacro(
            "package://poc_thyssen/models/ur10_femto_bolt.urdf.xacro"
        )

    mode = rospy.node.get_parameter_or(
        "mode", Parameter("mode", value="standalone")
    ).value

    if mode == "standalone":
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
            target_frequency=80, prediction_horizon=30
        ),
    )
    giskard.live()


if __name__ == "__main__":
    main()
