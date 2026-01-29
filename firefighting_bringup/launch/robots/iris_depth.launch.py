import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_state_pub_with_bridge(context):
    pkg_fire_desc = get_package_share_directory("firefighting_iris_description")
    pkg_bringup = get_package_share_directory("firefighting_bringup")

    # Load your specific iris_depth model
    sdf_file = os.path.join(pkg_fire_desc, "models", "iris_depth", "model.sdf")
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Dynamic SDF Path resolution for Gazebo sensors
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
        if "SDF_PATH" in os.environ:
            os.environ["SDF_PATH"] = os.environ["SDF_PATH"] + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # RobotStatePublisher publishes TF from your SDF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_desc, "frame_prefix": ""}],
    )

    # Bridge using your depth_bridge.yaml configuration
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(pkg_bringup, "config", "depth_bridge.yaml"),
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        output="screen",
    )

    # Relay Gazebo TF to /tf
    topic_tools_tf = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/gz/tf", "/tf"],
        output="screen",
        respawn=False,
        condition=IfCondition(LaunchConfiguration("use_gz_tf")),
    )

    event = RegisterEventHandler(
        OnProcessStart(target_action=bridge, on_start=[topic_tools_tf])
    )

    return [robot_state_publisher, bridge, event]

def generate_launch_description():
    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    use_gz_tf = DeclareLaunchArgument(
        "use_gz_tf", default_value="true", description="Use Gazebo TF."
    )

    # Updated ArduPilot SITL Include using dds_udp pattern
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ardupilot_sitl"),
                "launch",
                "sitl_dds_udp.launch.py", # Using the UDP specific launch
            ])
        ]),
        launch_arguments={
            "transport": "udp4",           # Set according to reference
            "port": "2019",                # Set according to reference
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "json",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            # Concatenate default params and dds_udp parameters
            "defaults": os.path.join(pkg_ardupilot_sitl, "config", "default_params", "gazebo-iris.parm")
                        + "," + 
                        os.path.join(pkg_ardupilot_sitl, "config", "default_params", "dds_udp.parm"),
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    opfunc_robot_state_publisher = OpaqueFunction(function=launch_state_pub_with_bridge)

    return LaunchDescription([
        use_gz_tf,
        sitl_dds,
        opfunc_robot_state_publisher,
    ])