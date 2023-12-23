from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_ = 'ros_gz_example_application'
    feedback_controller_node = Node(
        package= package_,
        executable= 'smore_feedback_controller_node',
        output='screen'
    )

    return LaunchDescription([
        feedback_controller_node,
    ])
