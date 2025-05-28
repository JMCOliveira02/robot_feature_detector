import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    world_package_dir = get_package_share_directory('robot_worlds')
    this_package_dir = get_package_share_directory('robot_feature_detector')
    robot_description_path = os.path.join(world_package_dir, 'urdf', 'robot.urdf')
    world = "iilab"
    world_path = "worlds/" + world + "/" + world + ".wbt"
    map_image_path = os.path.join(world_package_dir, 'maps', world, world + '.yaml')
    #map_features_path = os.path.join(world, 'feature_maps', world + '.yaml')

    my_robot_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    webots = WebotsLauncher(
        world=os.path.join(world_package_dir, world_path)
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
        prefix="gnome-terminal --tab --",
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
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "lidar2D"]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_image_path}],
        output='screen'
    )
    ## Map server lifecycle manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'autostart': True, 'node_names': ['map_server']}],
        output='screen'
    )

    return LaunchDescription([
        corner_detector,
        webots,
        my_robot_driver,
        tf_static_odom,
        tf_static_lidar,
        teleop,
        rviz,
        map_server,
        lifecycle_manager
    ])