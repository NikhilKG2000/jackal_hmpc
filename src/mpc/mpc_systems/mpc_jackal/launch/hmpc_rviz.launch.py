from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    jackal_share = Path(get_package_share_directory("mpc_jackal"))
    sim_share = Path(get_package_share_directory("simple_sim"))
    map_share = Path(get_package_share_directory("occupancygrid_creator"))

    def include(path, arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(path)),
            launch_arguments=(arguments or {}).items(),
        )

    initial_state_arguments = [
        DeclareLaunchArgument("x_init", default_value="0.0"),
        DeclareLaunchArgument("y_init", default_value="0.0"),
        DeclareLaunchArgument("psi_init", default_value="0.0"),
        DeclareLaunchArgument("v_init", default_value="0.0"),
    ]

    simulator = include(
        sim_share / "launch" / "simple_sim.launch.py",
        {
            name: LaunchConfiguration(name)
            for name in ("x_init", "y_init", "psi_init", "v_init")
        },
    )

    occupancy_grid = include(
        map_share / "launch" / "occupancygrid.launch.py"
    )

    controllers = include(
        jackal_share / "launch" / "hmpc_simplesim.launch.py"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            str(jackal_share / "rviz" / "hmpc_jackal.rviz"),
        ],
        output="screen",
    )

    return LaunchDescription(
        initial_state_arguments
        + [occupancy_grid, simulator, controllers, rviz]
    )