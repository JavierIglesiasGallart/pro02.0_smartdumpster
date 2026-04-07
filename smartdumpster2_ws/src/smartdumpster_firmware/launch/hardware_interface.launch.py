import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    smartdumpster_description_dir = get_package_share_directory("smartdumpster_description")

    robot_description = ParameterValue(Command([
        "xacro ", 
        os.path.join(smartdumpster_description_dir, "urdf", "sdumpster.urdf.xacro"),
        " is_sim:=False",        
        ]), 
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("smartdumpster_controller"),
                "config",
                "sdumpster_controllers.yaml"
            )
        ]
    )

    imu_driver_node = Node(
        package="smartdumpster_firmware",
        executable="mpu6050_driver.py"
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        imu_driver_node
    ])