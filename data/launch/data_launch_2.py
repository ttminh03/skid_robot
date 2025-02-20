from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        IncludeLaunchDescription(
            FindPackageShare('low_control').find('low_control') + '/launch/low_level_control.launch.xml'
        ),
        IncludeLaunchDescription(
            FindPackageShare('low_control').find('low_control') + '/launch/ps3.launch.xml'
        ),
        IncludeLaunchDescription(
            FindPackageShare('microstrain_inertial_driver').find('microstrain_inertial_driver') + '/launch/microstrain_launch.py'
        ),
        IncludeLaunchDescription(
            FindPackageShare('velodyne').find('velodyne') + '/launch/velodyne-all-nodes-VLP16-launch.py'
        ),
        Node(
            package='data',
            executable='odom3',
            name='odom3',
            parameters=[
                {'left_topic': 'wheel_data_left'},
                {'right_topic': 'wheel_data_right'},
                {'output_topic': '/odom'},
                {'imu_topic': '/imu/data'},  # Thêm tham số imu_topic cho node odom2
                {'khoang_cach_banh': 0.5},
                {'r_banh': 0.1}
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_velodyne_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_velodyne_tf',
            arguments=['0.15', '0', '0.3', '0', '0', '0', 'base_link', 'velodyne']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_tf',
            arguments=['0', '0', '0.', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', FindPackageShare('slam').find('slam') + '/config/slam_toolbox.rviz']
        # ),
    ])
