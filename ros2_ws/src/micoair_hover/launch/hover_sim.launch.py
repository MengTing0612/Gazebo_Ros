"""Launch the PX4 offboard hover controller after SITL and DDS are running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("sdk_mode", default_value="disabled"),
        DeclareLaunchArgument("sdk_resume", default_value="false"),
        DeclareLaunchArgument("sdk_timeout_ms", default_value="10000"),
        DeclareLaunchArgument("target_altitude_m", default_value="2.0"),
        DeclareLaunchArgument("sdk_residual_start_weight", default_value="1.0"),
        DeclareLaunchArgument("sdk_residual_hold_s", default_value="2.0"),
        DeclareLaunchArgument("sdk_residual_decay_s", default_value="20.0"),
        DeclareLaunchArgument("sdk_residual_final_weight", default_value="0.0"),
        DeclareLaunchArgument("sitl_visual_demo_enabled", default_value="false"),
        DeclareLaunchArgument("sitl_visual_demo_hold_s", default_value="8.0"),
        DeclareLaunchArgument("sitl_visual_demo_decay_s", default_value="14.0"),
        DeclareLaunchArgument("sitl_visual_demo_horizontal_mps", default_value="0.55"),
        DeclareLaunchArgument("sitl_visual_demo_vertical_mps", default_value="0.12"),
        DeclareLaunchArgument("sitl_visual_demo_period_s", default_value="7.0"),
        DeclareLaunchArgument(
            "vehicle_local_position_topic",
            default_value="/fmu/out/vehicle_local_position_v1",
        ),
        DeclareLaunchArgument(
            "vehicle_status_topic",
            default_value="/fmu/out/vehicle_status_v1",
        ),
        Node(
            package="micoair_hover",
            executable="offboard_hover",
            name="offboard_hover",
            output="screen",
            parameters=[{
                "auto_arm": LaunchConfiguration("auto_arm"),
                "sdk_mode": LaunchConfiguration("sdk_mode"),
                "sdk_resume": LaunchConfiguration("sdk_resume"),
                "sdk_timeout_ms": LaunchConfiguration("sdk_timeout_ms"),
                "target_altitude_m": LaunchConfiguration("target_altitude_m"),
                "sdk_residual_start_weight": LaunchConfiguration("sdk_residual_start_weight"),
                "sdk_residual_hold_s": LaunchConfiguration("sdk_residual_hold_s"),
                "sdk_residual_decay_s": LaunchConfiguration("sdk_residual_decay_s"),
                "sdk_residual_final_weight": LaunchConfiguration("sdk_residual_final_weight"),
                "sitl_visual_demo_enabled": LaunchConfiguration("sitl_visual_demo_enabled"),
                "sitl_visual_demo_hold_s": LaunchConfiguration("sitl_visual_demo_hold_s"),
                "sitl_visual_demo_decay_s": LaunchConfiguration("sitl_visual_demo_decay_s"),
                "sitl_visual_demo_horizontal_mps": LaunchConfiguration(
                    "sitl_visual_demo_horizontal_mps",
                ),
                "sitl_visual_demo_vertical_mps": LaunchConfiguration(
                    "sitl_visual_demo_vertical_mps",
                ),
                "sitl_visual_demo_period_s": LaunchConfiguration("sitl_visual_demo_period_s"),
                "vehicle_local_position_topic": LaunchConfiguration(
                    "vehicle_local_position_topic",
                ),
                "vehicle_status_topic": LaunchConfiguration("vehicle_status_topic"),
            }],
        ),
    ])
