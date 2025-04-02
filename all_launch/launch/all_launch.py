from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_tutorial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('urdf_tutorial'), 'launch', 'robot.launch.py')
        )
    )

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar.py')
        )
    )

    realsense2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),'launch','rs_launch.py')
        )
    )
    imu_node = ExecuteProcess(
        cmd=['ros2', 'run', 'imu_pkg', 'imu_publisher'],
        output='screen'
    )

    return LaunchDescription([
        urdf_tutorial_launch,
        ydlidar_launch,
        imu_node,
        realsense2_launch
    ])
