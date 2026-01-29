from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pkg_bringup = get_package_share_directory("firefighting_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_fire_desc = get_package_share_directory("firefighting_iris_description")
    pkg_ardupilot_gazebo = get_package_share_directory("ardupilot_gazebo")

    # Build GZ_SIM_RESOURCE_PATH with both model roots
    gz_paths = []
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing:
        gz_paths.append(existing)

    # Your description package (iris_depth, iris_with_standoffs)
    gz_paths.append(str(Path(pkg_fire_desc)))

    # ArduPilot Gazebo models (runway, original iris, etc.)
    gz_paths.append(str(Path(pkg_ardupilot_gazebo)))

    os.environ["GZ_SIM_RESOURCE_PATH"] = ":".join(gz_paths)

    # Optionally propagate to SDF_PATH for sdformat_urdf (RSP, RViz)
    gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]
    existing_sdf = os.environ.get("SDF_PATH", "")
    if existing_sdf:
        os.environ["SDF_PATH"] = existing_sdf + ":" + gz_sim_resource_path
    else:
        os.environ["SDF_PATH"] = gz_sim_resource_path

    # Iris (SITL + RSP + bridge)
    iris = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("firefighting_bringup"),
                        "launch",
                        "robots",
                        "iris_depth.launch.py",
                    ]
                ),
            ]
        )
    )


    # Gazebo server: fire_demo_world.sdf
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            + str(Path(pkg_fire_desc) / "worlds" / "fire_demo_world.sdf")
        }.items(),
    )

    # Gazebo GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )


    sim_fire = Node(
        package='firefighting_pkg',
        executable='sim_depth_fire_node',
        name='sim_depth_fire_node',
        output='screen'
    )
    


    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(Path(pkg_bringup) / "rviz" / "iris.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    
  

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            iris,
            gz_sim_server,
            gz_sim_gui,
            sim_fire,
            rviz,
            
        ]
    )

"""
    sim_yolo = Node(
        package='firefighting_pkg',
        executable='sim_yolo_node',
        name='sim_yolo_node',
        output='screen'
    )
        
     mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('firefighting_bringup'), 
            'config/mavros/mavros_node.yaml'
        ])]
    )

      mission = Node(
        package='firefighting_pkg',
        executable='simple_mission_node',
        name='simple_fire_mission',
        output='screen'
    )


    """