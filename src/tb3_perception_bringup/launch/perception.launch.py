"""
Launch file for perception.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Function to generate launch description.
    """

    # Paths and parameters
    world = os.path.join(
        get_package_share_directory('tb3_perception_bringup'),
        'worlds',
        'perception_world.world'
    )

    models_path = os.path.join(
        get_package_share_directory('tb3_perception_bringup'),
        'models'
    )

    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.5')
    y_pose = LaunchConfiguration('y_pose', default='-0.2')

    # Including launch descriptions
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Perception nodes
    detector_node = Node(
        name='detector_node',
        package='tb3_perception',
        executable='detector_node',
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )

    fusion_node = Node(
        name='fusion_node',
        package='tb3_perception',
        executable='fusion_node',
        remappings=[
            ('camera/camera_info', '/camera/camera_info'),
            ('scan', '/scan'),
        ]
    )

    approach_node = Node(
        name='approach_node',
        package='tb3_perception',
        executable='approach_node'
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gazebo_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(detector_node)
    ld.add_action(fusion_node)
    ld.add_action(approach_node)

    return ld