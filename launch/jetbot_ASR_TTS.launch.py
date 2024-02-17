import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

#
# ros2 launch jetbot_voice jetbot_ASR_TTS.launch.py param_file:=./jetbot_voice/param/jetbot_voice_params.yaml output_device:=10
#

def generate_launch_description():

    # Declare the audio 'output_device' argument
    output_device_arg = DeclareLaunchArgument(
        'output_device',
        default_value='10',
        description='Output device ID'
    )

    param_file_cmd = DeclareLaunchArgument(
        'param_file', default_value='./jetbot_voice/param/jetbot_voice_params.yaml')

    # Start the ROS2 node receive Jetson ASR transcript 
    # filter with keyword list then set parameter to the target two wheel bot.
    start_jetbot_ASR_node_cmd = Node(
        package='jetbot_voice',
        executable='jetbot_ASR',
        output="screen",
        parameters=[LaunchConfiguration('param_file')]
        )

    # Use jetson_voice sample code to fine the audio output_device id
    # examples/asr.py --list-devices
    start_jetbot_TTS_node_cmd = Node(
        package='jetbot_voice',
        executable='jetbot_TTS',
        output="screen",
        parameters=[
            LaunchConfiguration('param_file'),
            {'output_device': LaunchConfiguration('output_device')}
            ]
        )
    
    ld = LaunchDescription()

    # Add any actions
    ld.add_action(output_device_arg)
    ld.add_action(param_file_cmd)
    

    ld.add_action(start_jetbot_ASR_node_cmd)
    ld.add_action(start_jetbot_TTS_node_cmd)

    return ld