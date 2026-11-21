"""ManipulatorCore demo: robot_state_publisher, simulator, planner and (optionally) RViz.

Usage:
  ros2 launch manipulatorcore_ros demo.launch.py [robot:=redundant7r] [scenario:=tabletop_reach] [rviz:=true]
"""

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

PACKAGE = 'manipulatorcore_ros'


def _rviz(context, *args, **kwargs):
    try:
        get_package_share_directory('rviz2')
    except PackageNotFoundError:
        return [LogInfo(msg='rviz:=true was requested but rviz2 is not installed; continuing without RViz.')]
    config = PathJoinSubstitution([FindPackageShare(PACKAGE), 'rviz', 'manipulatorcore.rviz'])
    return [Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', config], output='screen')]


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    scenario = LaunchConfiguration('scenario')
    rviz = LaunchConfiguration('rviz')

    export_urdf = PathJoinSubstitution([FindPackagePrefix(PACKAGE), 'lib', PACKAGE, 'export_urdf'])
    robot_description = ParameterValue(Command([export_urdf, ' --robot ', robot]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='redundant7r',
                              description='Built-in robot: redundant7r, industrial6r, planar2r or planar3r'),
        DeclareLaunchArgument('scenario', default_value='tabletop_reach',
                              description='Planning scenario of the redundant7r robot, or "none"'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz if it is installed'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package=PACKAGE,
            executable='manipulatorcore_simulator',
            name='manipulatorcore_simulator',
            output='screen',
            parameters=[{'robot': robot, 'scenario': scenario}],
        ),
        Node(
            package=PACKAGE,
            executable='manipulatorcore_planner',
            name='manipulatorcore_planner',
            output='screen',
            parameters=[{'robot': robot, 'scenario': scenario}],
        ),
        OpaqueFunction(function=_rviz, condition=IfCondition(rviz)),
    ])
