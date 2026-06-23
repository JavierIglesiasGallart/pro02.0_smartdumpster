import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node


def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )

    use_python = LaunchConfiguration("use_python")

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.1175",
            "--qx", "1", "--qy", "0", "--qz", "0.0", "--qw", "0",
            "--frame-id", "base_footprint_ekf",
            "--child-frame-id", "imu_link_ekf"
        ]
    )

    robot_localization_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("smartdumpster_localization"), "config", "ekf.yaml")
        ],
        remappings=[(
            '/odometry/filtered', '/odometry/local'
        )]
    )

    robot_localization_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory("smartdumpster_localization"), "config", "ekf.yaml")
        ],
        remappings=[(
            '/odometry/filtered', '/odometry/global'
        )]
    )
    
    node_navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory("smartdumpster_localization"), "config", "ekf.yaml")
        ],
        remappings=[
            ('/gps/fix', '/gps/fix'),               # Entrada: GPS remapeado del puente
            ('/imu', '/zed/imu/out'),               # Entrada: IMU de la cámara ZED
            ('/odometry/filtered', '/odometry/local') # Entrada: Odometría local filtrada
        ]
    )

    imu_republisher_py = Node(
        package="smartdumpster_localization",
        executable="imu_republisher.py",
        condition=IfCondition(use_python)
    )

    imu_republisher = Node(
        package="smartdumpster_localization",
        executable="imu_republisher",
        condition=UnlessCondition(use_python)
    )

    return LaunchDescription([
        use_python_arg,
        #static_transform_publisher,
        robot_localization_local,
        #imu_republisher_py,
        #imu_republisher,
        robot_localization_global,
        node_navsat_transform
    ])