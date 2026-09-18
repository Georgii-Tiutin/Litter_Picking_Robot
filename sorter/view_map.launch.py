"""Show the RECORDED map and localise the robot in it.

Deliberately NOT using M3Pro_navigation/localization.launch.py: it sets use_sim_time:=True on
the lifecycle manager (same defect class as the global costmap) AND it reads params from
yahboom_M3_nav2_bringup, a package that is not installed on this robot - so it cannot run.
The amcl block in yahboom_M3Pro.yaml is present and already has use_sim_time: false.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

MAP    = "/home/jetson/maps/office_2026-09-12.yaml"
PARAMS = "/home/jetson/M3Pro_ws/install/M3Pro_navigation/share/M3Pro_navigation/param/yahboom_M3Pro.yaml"

def generate_launch_description():
    return LaunchDescription([
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen",
             parameters=[{"yaml_filename": MAP}, {"use_sim_time": False},
                         {"topic_name": "map"}, {"frame_id": "map"}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[PARAMS, {"use_sim_time": False}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": False}, {"autostart": True},
                         {"node_names": ["map_server", "amcl"]}]),
    ])
