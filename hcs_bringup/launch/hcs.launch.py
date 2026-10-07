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

        # 中间件：默认沿用环境（不设 = Jazzy 默认的 Fast DDS）。rmw:=zenoh 时本机起一个 zenoh 路由
        # （监听 tcp/[::]:7447，笔记本的路由连它），executor 改用 rmw_zenoh_cpp。只影响话题、参数这些
        # 非实时线程上的东西：控制线程不碰中间件，选哪个都一样。
        rmw = LaunchConfiguration("rmw").perform(context)
        executor_env = {}
        if rmw == "zenoh":
            executor_env["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
            entities.append(LogInfo(msg="Middleware: rmw_zenoh_cpp, router on tcp/[::]:7447"))
            entities.append(
                Node(
                    package="rmw_zenoh_cpp",
                    executable="rmw_zenohd",
                    name="zenoh_router",
                    additional_env=executor_env,
                    respawn=True,
                    respawn_delay=1.0,
                    output="log",
                )
            )
        elif rmw != "default":
            raise ValueError(f"rmw:={rmw} is not one of: default, zenoh")

        entities.append(
            Node(
                package="hcs_executor",
                executable="hcs_executor",
                parameters=parameter_files,
                additional_env=executor_env,
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
            DeclareLaunchArgument(
                "rmw",
                default_value="default",
                description="'default' keeps the environment's middleware; 'zenoh' starts a "
                "zenoh router and runs the executor on rmw_zenoh_cpp (telemetry over Wi-Fi)",
            ),
            MyLaunchDescriptionEntity(),
        ]
    )

    return ld
