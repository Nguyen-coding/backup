from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth_obstacle_detect_ros',
            executable='depth_obstacle_detect_ros_node',
            name='obstacle_detection_node',
            namespace='cam1',
            parameters=[{
                'depth_topic':'/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic':'/camera/camera/color/camera_info',
                'detect_topic':'/camera/depth_obstacle_detect/image',
                'cam_id':'cam1',
                'verbose': True,
                'width_regions':12,
                'height_regions':9,
                'obstacle_range_limit':1.0,
                'obstacle_state_topic':'obstacle_state',
                'max_distance_for_visualization':16
            }]
        )
    ])
