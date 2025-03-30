import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    urdf_file = "/home/cv25/ros2_ws/src/urdf_tutorial/urdf/robot.urdf"
    
    # URDF 파일 읽기
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Joint State Publisher (GUI 사용 시 아래 주석 해제)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        # GUI를 쓰고 싶다면 아래를 대신 사용
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
        # robot_state_publisher: 로봇 상태 및 TF 브로드캐스트
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # RViz2: RViz를 통한 시각화
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'
        # ),
        # Gazebo 시뮬레이터 실행
        # gazebo_launch,
        # Gazebo에 로봇 스폰
        # spawn_entity
    ])

