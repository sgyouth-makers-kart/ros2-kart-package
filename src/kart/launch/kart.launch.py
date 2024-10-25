from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kart',
            executable='main.py',
            name='main',
            output='screen',
        ),
        # Node(
            # package='kart',
            # executable='pump_DC.py',
            # name='pump_dc',
            # output='screen'
        # ),
        Node(
            package='kart',
            executable='wheel_DC.py',
            name='wheel_dc',
            output='screen'
        ),
        # Node(
            # package='kart',
            # executable='steering_servo.py',
            # name='steering_servo',
            # output='screen'
        # ),
    ])


