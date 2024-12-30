import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory('state_estimator')
    config_file = os.path.join(package_share_dir, 'config', 'state_estimator_params.yaml')
    
    return LaunchDescription([
        # State Estimator Node
        Node(
            package='state_estimator',
            executable='state_estimator',
            name='state_estimator_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/tracked_pose', '/pf/viz/inferred_pose'),
                ('/imu', '/imu'),
                ('/odom', '/odom'),
            ]
        )
    ])
