"""Drone role: Xbox-gated mission, RGB fixture perception, optional physical UART.
The Gazebo suite launches these processes separately to retain original logs.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    arg=LaunchConfiguration
    use_sim=ParameterValue(arg('use_sim_time'),value_type=bool)
    common=dict(package='qcopter_autonomy',namespace=arg('namespace'),output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value='drone'),
        DeclareLaunchArgument('panel_model',description='Trained 320x320 raw panel ONNX file'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('known_fixture_model_confirmed',default_value='false'),
        DeclareLaunchArgument('enable_uart',default_value='false'),
        DeclareLaunchArgument('uart_port',default_value=''),
        Node(**common,executable='pilot_mission_node',parameters=[{'use_sim_time':use_sim}]),
        Node(**common,executable='yolo_node',parameters=[{'use_sim_time':use_sim,'onnx_model':arg('panel_model'),'input_size':320}]),
        Node(**common,executable='panel_pose_node',parameters=[{'use_sim_time':use_sim,'known_fixture_model_confirmed':ParameterValue(arg('known_fixture_model_confirmed'),value_type=bool)}]),
        Node(**common,executable='uart_bridge_node',condition=IfCondition(arg('enable_uart')),parameters=[{'port':arg('uart_port')}]),
    ])
