import launch
import os, sys, yaml
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

DEFAULT_ID=1
DEFAULT_ENV='phys'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    #TODO: Note this doesn't work when passed from higher-level launch file
    launch_arg_drone_id = DeclareLaunchArgument(
      'id', default_value=str(DEFAULT_ID)
    )

    launch_arg_device_type = DeclareLaunchArgument(
      'device_type', default_value="drone"
    )

    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments  
    id = LaunchConfiguration('id')
    device_type = LaunchConfiguration('device_type')

    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = arg.split(":=")[1]

    ## GET PARAMETERS
    config = None

    if env=="sim":
      config = os.path.join(
        get_package_share_directory('multi_drone_slung_load'),
        'config',
        'sim.yaml'
        )
    elif env=="phys":
       config = os.path.join(
        get_package_share_directory('multi_drone_slung_load'),
        'config',
        'phys.yaml'
        ) 

    # Set up launch description to launch measurement node with arguments
    launch_description = [
        launch_arg_drone_id,
        launch_arg_device_type,
        launch_arg_sim_phys,
        Node(
            package='multi_drone_slung_load_cpp',
            executable='pixhawk',
            namespace=PythonExpression(["'/px4_' + str(", id, ")"]),
            name=PythonExpression(["'pixhawk_", device_type, "_' + str(", id, ")"]),
            output='screen',
            parameters=[config]
        )]

    ## LAUNCH
    return LaunchDescription(launch_description)