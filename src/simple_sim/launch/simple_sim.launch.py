from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("x_init", default_value="0.0"),
        DeclareLaunchArgument("y_init", default_value="0.0"),
        DeclareLaunchArgument("psi_init", default_value="0.0"),
        DeclareLaunchArgument("v_init", default_value="0.0"),
        DeclareLaunchArgument(
            "run_event_based", default_value="true"
        ),
        DeclareLaunchArgument("model_dt", default_value="0.05"),
        DeclareLaunchArgument("steps", default_value="1"),
        DeclareLaunchArgument("model_rate", default_value="20.0"),
        DeclareLaunchArgument(
            "add_timing_variance", default_value="false"
        ),
    ]

    simulator = Node(
        package="simple_sim",
        executable="jackal_model_node",
        name="dyn_model_node",
        output="screen",
        arguments=[
            LaunchConfiguration("x_init"),
            LaunchConfiguration("y_init"),
            LaunchConfiguration("psi_init"),
            LaunchConfiguration("v_init"),
            LaunchConfiguration("run_event_based"),
            LaunchConfiguration("model_dt"),
            LaunchConfiguration("steps"),
            LaunchConfiguration("model_rate"),
            LaunchConfiguration("add_timing_variance"),
        ],
        on_exit=Shutdown(reason="The Jackal simulator exited"),
    )

    return LaunchDescription(arguments + [simulator])