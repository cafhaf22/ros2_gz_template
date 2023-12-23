from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_ = 'ros_gz_example_application'
    open_loop_controller_node = Node(
        package= package_,
        executable= 'smore_open_loop_controller_node',
        output='screen'
    )

    return LaunchDescription([
        open_loop_controller_node,
    ])
