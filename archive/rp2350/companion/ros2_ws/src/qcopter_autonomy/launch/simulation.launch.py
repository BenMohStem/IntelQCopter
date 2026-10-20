from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='qcopter_autonomy', executable='navigator_node',
        parameters=[{'use_sim_time': True, 'obstacles': [3.,0.,.65,5.5,-.8,.65]}], output='screen')])
