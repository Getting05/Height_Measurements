from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("goal_points_path", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value="/aft_mapped_in_map"),
        DeclareLaunchArgument("goal_yaw_topic", default_value="/parkour/goal_yaw"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("publish_rate", default_value="50.0"),
        DeclareLaunchArgument("goal_reach_threshold", default_value="0.2"),
        DeclareLaunchArgument("reach_goal_delay", default_value="0.1"),
        DeclareLaunchArgument("base_to_odom_x", default_value="0.16266"),
        DeclareLaunchArgument("base_to_odom_y", default_value="0.0"),
        DeclareLaunchArgument("base_to_odom_z", default_value="0.11703"),
    ]

    node = Node(
        package="height_measurements",
        executable="parkour_goal_manager_node",
        name="parkour_goal_manager_node",
        output="screen",
        parameters=[
            {
                "goal_points_path": LaunchConfiguration("goal_points_path"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "goal_yaw_topic": LaunchConfiguration("goal_yaw_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "publish_rate": ParameterValue(LaunchConfiguration("publish_rate"), value_type=float),
                "goal_reach_threshold": ParameterValue(
                    LaunchConfiguration("goal_reach_threshold"), value_type=float
                ),
                "reach_goal_delay": ParameterValue(
                    LaunchConfiguration("reach_goal_delay"), value_type=float
                ),
                "base_to_odom_x": ParameterValue(
                    LaunchConfiguration("base_to_odom_x"), value_type=float
                ),
                "base_to_odom_y": ParameterValue(
                    LaunchConfiguration("base_to_odom_y"), value_type=float
                ),
                "base_to_odom_z": ParameterValue(
                    LaunchConfiguration("base_to_odom_z"), value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(args + [node])
