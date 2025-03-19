from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Set the TurtleBot 3 model
    turtlebot3_model = 'waffle_pi'

    # Export the TurtleBot 3 model
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value=turtlebot3_model
    )

    # Launch Gazebo with a predefined world
    launch_gazebo = ExecuteProcess(
        cmd=[
            'ros2',
            'launch',
            'turtlebot3_gazebo',
            'turtlebot3_world.launch.py'
        ],
        output='screen'
    )

    # Start your ROS 2 node (e.g., vff_avoidance)
    start_vff_avoidance_node = Node(
        package='vff_avoidance',  # Replace with your package name
        executable='vff_avoidance',  # Replace with your node executable name
        name='vff_avoidance_node',  # Node name
        output='screen'
    )

    # Create the launch description
    return LaunchDescription([
        set_turtlebot3_model,
        launch_gazebo,
        start_vff_avoidance_node
    ])
