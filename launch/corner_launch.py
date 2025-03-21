import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    world_package_dir = get_package_share_directory('robot_localization_package')
    this_package_dir = get_package_share_directory('robot_feature_detector')
    robot_description_path = os.path.join(world_package_dir, 'resource', 'my_robot.urdf')

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    webots = WebotsLauncher(
        world=os.path.join(world_package_dir, 'worlds', 'epuck_world.wbt')
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(this_package_dir, "rviz", "corners_lidar.rviz")],
        output="screen"
    )

    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="gnome-terminal --",
    )

    corner_detector = Node(
        package="robot_feature_detector",
        executable="corners",
        name="corners",
        output="screen"
    )

    tf_static_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    tf_static_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_broadcaster",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar2D"]
    )


    return LaunchDescription([
        corner_detector,
        webots,
        my_robot_driver,
        tf_static_odom,
        tf_static_lidar,
        teleop,
        rviz
    ])