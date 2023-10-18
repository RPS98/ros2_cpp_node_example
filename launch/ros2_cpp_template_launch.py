""" Launch ros2_cpp_template platform node """

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare


def get_node(context, *args, **kwargs):
    """ Return a ros2_cpp_template node """
    node = Node(
        package="ros2_cpp_template",
        executable="ros2_cpp_template_node",
        name=LaunchConfiguration('name'),
        namespace=LaunchConfiguration('namespace'),
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": LaunchConfiguration('use_sim_time')
            },
            LaunchConfiguration('config_file')
        ]
    )
    return [node]


def generate_launch_description():
    """ Entrypoint for launch file """

    config_file = PathJoinSubstitution([
        FindPackageShare('ros2_cpp_template'),
        'config', 'config_file.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='ros2_cpp_template_node'),
        DeclareLaunchArgument(
            'name', default_value='ros2_cpp_template'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_file',
                              default_value=config_file,
                              description='Configuration file'),
        OpaqueFunction(function=get_node)
    ])
