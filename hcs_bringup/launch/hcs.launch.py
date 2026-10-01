from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self, context: "LaunchContext"
    ) -> Optional[List["LaunchDescriptionEntity"]]:
        entities = []

        robot_config = LaunchConfiguration("robot").perform(context)
        if robot_config.startswith("auto."):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        entities.append(
            LogInfo(
                msg=f"Starting HCS on robot '{robot_config}'{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml"
            )
        )

        # 先机器、后机器人：机器人 yaml 里的同名键覆盖机器配置。machine:=none 不加载。
        config_dir = os.path.join(FindPackageShare("hcs_bringup").perform(context), "config")
        machine_name = LaunchConfiguration("machine").perform(context)
        parameter_files = []
        if machine_name != "none":
            parameter_files.append(os.path.join(config_dir, "machine", machine_name + ".yaml"))
            entities.append(LogInfo(msg=f"Machine config: machine/{machine_name}.yaml"))
        parameter_files.append(os.path.join(config_dir, robot_name + ".yaml"))

        entities.append(
            Node(
                package="hcs_executor",
                executable="hcs_executor",
                parameters=parameter_files,
                respawn=True,
                respawn_delay=1.0,
                output="log",  # stdout and stderr are logged to launch log file and stderr to the screen.
            )
        )

        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "machine",
                default_value="tl101",
                description="config/machine/<machine>.yaml, loaded before the robot yaml; "
                "'none' skips it",
            ),
            MyLaunchDescriptionEntity(),
        ]
    )

    return ld
