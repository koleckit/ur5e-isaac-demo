"""Offline MoveIt/cuMotion planning only; no trajectory execution nodes."""
from pathlib import Path
import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    demo = Path('/demo')
    ur = Path(get_package_share_directory('ur_moveit_config'))
    backend = Path(get_package_share_directory('isaac_ros_cumotion'))
    plugin = Path(get_package_share_directory('isaac_ros_cumotion_moveit'))
    urdf_file = os.environ.get('UR_DEMO_URDF', '/demo/ur5e.urdf')
    params = {
        'robot_description': Path(urdf_file).read_text(),
        'robot_description_semantic': xacro.process_file(str(ur / 'srdf/ur.srdf.xacro'),
                                                        mappings={'name': 'ur5e'}).toxml(),
        'robot_description_kinematics': yaml.safe_load((ur / 'config/kinematics.yaml').read_text()),
        'robot_description_planning': yaml.safe_load((ur / 'config/joint_limits.yaml').read_text()),
        'planning_pipelines': ['isaac_ros_cumotion'],
        'default_planning_pipeline': 'isaac_ros_cumotion',
        'isaac_ros_cumotion': yaml.safe_load((plugin / 'config/isaac_ros_cumotion_planning.yaml').read_text()),
        'allow_trajectory_execution': False,
        'publish_robot_description': False,
        'publish_robot_description_semantic': False,
        'use_sim_time': False,
    }
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(backend / 'launch/isaac_ros_cumotion.launch.py')),
            launch_arguments={
                'cumotion_action_server.urdf_file_path': urdf_file,
                'cumotion_action_server.xrdf_file_path':
                    '/opt/ros/lyrical/share/isaac_ros_cumotion_robot_description/xrdf/ur5e.xrdf',
                'cumotion_action_server.read_esdf_world': 'false',
                'cumotion_action_server.interpolation_dt': os.environ.get('UR_DEMO_INTERPOLATION_DT', '0.05'),
            }.items()),
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group',
             output='screen', parameters=[params]),
    ])
