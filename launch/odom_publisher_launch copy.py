from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        output='screen'
    )

    imu_reader_node = Node(
        package='imu_to_odom',
        executable='imureader',
        name='imureader',
        output='screen'
    )

    odom_publisher_node = Node(
        package='imu_to_odom',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen'
    )

    return LaunchDescription([
        ydlidar_launch,
        imu_complementary_filter_node,
        imu_reader_node,
        odom_publisher_node
    ])