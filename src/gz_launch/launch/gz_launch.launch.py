from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


rviz2_config_path = os.path.join(
    get_package_share_directory('gz_launch'),
    'config',
    'crawler_sim.rviz',
),

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
        ),

        Node(
            package='joy2twist',
            executable='joy2twist',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/image/depth@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/camera2/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                '/camera2/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera2/image/depth@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/world/default/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0.08', '0', '0', '0', 'base_link', 'crawler_robot/base_link/gpu_lidar',
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', rviz2_config_path,
            ],
        ),

    ])
