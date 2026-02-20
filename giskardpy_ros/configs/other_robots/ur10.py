"""
UR10 Giskard configuration and entry point.

Provides world, robot-interface, and standalone config classes for a UR10
robot with a Femto Bolt camera, as well as the ``main()`` ROS 2 entry point.
"""
from __future__ import annotations

from typing_extensions import TYPE_CHECKING, Optional

from giskardpy.model.world_config import WorldWithFixedRobot
from giskardpy_ros.configs.robot_interface_config import (
    RobotInterfaceConfig,
    StandAloneRobotInterfaceConfig,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.ur import UR10Bolt

if TYPE_CHECKING:
    from semantic_digital_twin.world import World


class WorldWithUR10Config(WorldWithFixedRobot):
    """
    World configuration for a UR10 robot with Femto Bolt camera.

    Fixed-base robot. Accepts URDF via argument; if not provided, reads
    from the ROS parameter server.
    """

    def __init__(self, urdf: Optional[str] = None):
        super().__init__(
            urdf=urdf,
            root_name=PrefixedName("map2"),
            urdf_view=UR10Bolt,
        )

    def setup_collision_config(self) -> None:
        pass

    def setup_world(self, robot_name: Optional[str] = None) -> None:
        """
        Set up the world and locate the UR10Bolt semantic annotation.

        :param robot_name: Unused, kept for interface compatibility.
        """
        super().setup_world()
        self.robot = self.world.get_semantic_annotations_by_type(UR10Bolt)[0]


class UR10VelocityInterface(RobotInterfaceConfig):
    """
    Robot interface for the UR10 in velocity control mode.

    Subscribes to ``/joint_states`` and publishes velocity commands on
    ``/forward_velocity_controller/commands``.
    """

    def setup(self) -> None:
        self.sync_joint_state_topic("/joint_states")
        joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.add_joint_velocity_group_controller(
            cmd_topic="/forward_velocity_controller/commands",
            connections=joints,
        )


class UR10StandAloneRobotInterfaceConfig(StandAloneRobotInterfaceConfig):
    """Standalone robot interface for the UR10 (simulation, no hardware)."""

    def __init__(self) -> None:
        super().__init__(
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
        )


def main() -> None:
    """
    Entry point for the UR10 Giskard node.

    Declares ROS parameters ``robot_description`` and ``mode``, builds the
    appropriate configuration, and starts the Giskard behaviour-tree loop.

    - ``mode="standalone"`` (default): :class:`StandAloneBTConfig` +
      :class:`UR10StandAloneRobotInterfaceConfig`
    - ``mode="hardware"``: :class:`ClosedLoopBTConfig` +
      :class:`UR10VelocityInterface`
    """
    from giskardpy.model.collision_world_syncer import CollisionCheckerLib
    from giskardpy.qp.qp_controller_config import QPControllerConfig
    from giskardpy_ros.configs.behavior_tree_config import (
        ClosedLoopBTConfig,
        StandAloneBTConfig,
    )
    from giskardpy_ros.configs.giskard import Giskard
    from giskardpy_ros.ros2 import rospy
    from giskardpy_ros.ros2.visualization_mode import VisualizationMode
    from giskardpy_ros.utils.utils import load_xacro
    from rclpy import Parameter
    from rclpy.exceptions import ParameterUninitializedException

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
