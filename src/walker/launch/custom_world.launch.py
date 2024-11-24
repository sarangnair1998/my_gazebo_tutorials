from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for enabling/disabling rosbag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
        description='Enable or disable rosbag recording'
    )

    # Gazebo process
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
            'src/walker/worlds/custom_world.world'
        ],
        output='screen'
    )

    # Walker node
    walker_node = Node(
        package='walker',
        executable='walker_node',
        name='walker_node',
        output='screen'
    )

    # Rosbag2 recording process
    rosbag_record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-x', '/camera/.*'],
        condition=IfCondition(LaunchConfiguration('record_bag')),
        output='screen'
    )

    return LaunchDescription([
        # Declare the rosbag recording argument
        record_bag_arg,
        # Launch Gazebo
        gazebo_process,
        # Launch the Walker node
        walker_node,
        # Conditionally start rosbag recording
        rosbag_record_process,
    ])
