from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    mobile_robot_control_node = Node(
            package='mobile_robot_control',
            executable='mobile_robot_control_node',
            name='robot_controll'
        )
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
        mobile_robot_control_node,
        joy_teleop_keymapping_node,
        joy_node,
    ])