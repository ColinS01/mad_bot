import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
import random

def generate_launch_description():
    pkg_mad_bot = get_package_share_directory('mad_bot')
    
    # Set the resource path for Gazebo models if needed
    model_path = PathJoinSubstitution([pkg_mad_bot, 'model'])
    world_file = os.path.join(pkg_mad_bot, 'world', 'world.sdf')
    bridge_file = os.path.join(pkg_mad_bot, 'config', 'ros_gz_bridge.yaml')

    paths = [
        'april_tag_1.sdf',
        'april_tag_2.sdf',
        'april_tag_3.sdf'
    ]
    selected_path = random.choice(paths)

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='car_world',
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration('world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        
        # Start Ignition Gazebo with a specific world
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '--render-engine ogre2'],
            output='screen',
            shell=True,
        ),

        # Spawn the robot model in the simulation
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-entity', 'mad_bot',
                '-file', os.path.join(pkg_mad_bot, 'model', 'mad_bot.sdf'),
                '-x', '0', 
                '-y', '0', 
                '-z', '0.1'
            ]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_april_tag',
            output='screen',
            arguments=[
                '-entity', 'april_tag',
                '-file', os.path.join(pkg_mad_bot, 'model', 'april_tag', selected_path),
                '-x', '2', 
                '-y', '0', 
                '-z', '1'
            ]
        ),
        # Robot State Publisher
        #Node(
        #    package='robot_state_publisher',
        #    executable='robot_state_publisher',
        #    name='robot_state_publisher',
        #    output='both',
        #    parameters=[
        #        {'robot_description': open(os.path.join(pkg_mad_bot, 'model', 'mad_bot.urdf')).read()},
        #        # Removed use_sim_time here
        #    ]
        #),

        # Launch ROS-GZ Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_file}],
        ),
        Node(
            package='mad_bot',
            executable='movement_controller',
            name='movement_controller',
            output='screen',
            # Removed use_sim_time here
        ),
        Node(
            package='mad_bot',
            executable='camera_controller',
            name='camera_controller',
            output='screen',
            # Removed use_sim_time here
        ),
    ])