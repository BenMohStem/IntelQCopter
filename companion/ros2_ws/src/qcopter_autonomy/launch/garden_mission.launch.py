from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('panel_model',description='320x320 raw YOLO panel ONNX path'),
        DeclareLaunchArgument('enable_simulation_missions',default_value='false'),
        DeclareLaunchArgument('known_fixture_model_confirmed',default_value='false'),
        Node(package='qcopter_autonomy',executable='yolo_node',parameters=[{
            'onnx_model':LaunchConfiguration('panel_model'),'input_size':320}],output='screen'),
        Node(package='qcopter_autonomy',executable='panel_pose_node',parameters=[{
            'known_fixture_model_confirmed':ParameterValue(LaunchConfiguration('known_fixture_model_confirmed'),value_type=bool)}],output='screen'),
        Node(package='qcopter_autonomy',executable='mission_node',parameters=[{
            'enable_simulation_missions':ParameterValue(LaunchConfiguration('enable_simulation_missions'),value_type=bool)}],output='screen')])
