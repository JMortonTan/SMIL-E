from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joy_teleop_keymapping_node = Node(
            package='joy_teleop_keymapping',
            executable='keymapping_node',
            name='keymap'
        )
    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy',
        )

    return LaunchDescription([
        joy_teleop_keymapping_node,
        joy_node,
    ])