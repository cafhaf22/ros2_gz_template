from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_ = 'ros_gz_example_application'
    convert_velocities_node = Node(
        package= package_,
        executable= 'smore_convert_velocities_node',
        output='screen'
    )

    smore_localiazation_node = Node(
        package= package_,
        executable='smore_localization_node',
        output='screen'
    )

    return LaunchDescription([
        smore_localiazation_node,
        convert_velocities_node,
    ])
