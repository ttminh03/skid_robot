from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', '/home/ttminh/Documents/skid_robot/src/mapping/config',
                '-configuration_basename', 'turtlebot3_lds_2d.lua'
            ],
            remappings=[
                ('scan', '/scan')  # Đảm bảo topic này đúng
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            remappings=[
                ('map', '/map')  # Thay đổi nếu cần
            ]
        ),
    #     Node(
	#     package='tf2_ros',
	#     executable='static_transform_publisher',
	#     name='static_transform_publisher',
	#     output='screen',
	#     arguments=['0', '1', '0', '0', '0', '0', 'map', 'odom']
	# ),

    ])
