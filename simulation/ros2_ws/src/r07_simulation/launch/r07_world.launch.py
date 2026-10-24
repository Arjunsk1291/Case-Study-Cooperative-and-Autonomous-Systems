from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = PathJoinSubstitution([
        FindPackageShare("r07_simulation"), "worlds", "dit_b_to_h.world"
    ])
    xacro_file = PathJoinSubstitution([
        FindPackageShare("r07_description"), "urdf", "r07_rover.urdf.xacro"
    ])
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world, "verbose": "true"}.items()
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="r07_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )
    spawn_robot = TimerAction(
        period=3.0,
        actions=[Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "r07",
                "-topic", "robot_description",
                "-x", "-11.0", "-y", "13.3", "-z", "0.02",
                "-Y", "-1.57079632679"
            ],
            output="screen"
        )]
    )
    return LaunchDescription([gazebo_launch, state_publisher, spawn_robot])
