import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    backend = LaunchConfiguration('backend')

    backend_launch_arg = DeclareLaunchArgument(
        'backend',
        default_value='cpp'
    )

    rviz = LaunchConfiguration('rviz')

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value='cpp'
    )
    
    crazyflie = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crazyflie'), 'launch'),
            '/launch.py']),
        launch_arguments={
            'backend': backend,
            'rviz': rviz
            }.items()
    )

    example_node = Node(
        package='crazyflie_leader_follower',
        executable='run',
        name='run',
        parameters=[{
            'use_sim_time': PythonExpression(["'", backend, "' == 'sim'"]),
        }]
    )

    return LaunchDescription([
        backend_launch_arg,
        rviz_launch_arg,
        crazyflie,
        example_node
    ])

