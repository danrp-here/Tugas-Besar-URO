import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('ROS_WORKSPACE', default=os.getcwd()),
        'src',
        'plane_kinematics_algorithm',
        'urdf',
        'plane_with_payload.urdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_path,
            description='Path to the URDF model of the plane and payload'
        ),
        LogInfo(msg="Starting Gazebo and spawning the plane..."),

        # Launch Gazebo and spawn the plane
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file', urdf_path, '-entity', 'plane_with_payload', '-x', '0', '-y', '0', '-z', '20'],
            output='screen'
        ),

        # Launch the kinematics controller for managing the payload drop
        Node(
            package='plane_kinematics_algorithm',
            executable='kinematics_controller',
            output='screen'
        ),
    ])

