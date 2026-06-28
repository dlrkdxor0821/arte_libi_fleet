"""libi 관제 RViz 뷰 — 맵 + navgraph + 로봇 3대(마커) 를 map 프레임에 띄운다.

run_sim.sh 의 rviz 창이 호출. slotcar 는 TF/RobotModel 이 없어, /robot_state 를
robot_markers.py 가 MarkerArray(/robot_markers)로 변환해 rviz 에 로봇을 표시한다.

  map_server       : new_map.yaml → /map (frame=map, latched)
  map→odom static  : map 프레임 생성(로봇 마커가 map 기준)
  show_navgraph    : /navgraph_markers (lane·vertex)   ← open-rmf-practice 재사용
  robot_markers    : /robot_state → /robot_markers (로봇 3대)
  rviz2            : scripts/sim/libi.rviz (Map + Navgraph + Robots, fixed=map)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SHOW_NAVGRAPH = os.path.expanduser(
    "~/personal_repo/open-rmf-practice/scripts/rmf/show_navgraph.py")


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    navgraph = LaunchConfiguration("navgraph")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("navgraph", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(package="nav2_map_server", executable="map_server", name="map_server",
             output="screen",
             parameters=[{"yaml_filename": map_yaml, "use_sim_time": use_sim_time}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_view", output="screen",
             parameters=[{"autostart": True, "node_names": ["map_server"],
                          "use_sim_time": use_sim_time}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="map_to_odom_static",
             arguments=["--frame-id", "map", "--child-frame-id", "odom"]),

        ExecuteProcess(cmd=["python3", SHOW_NAVGRAPH, navgraph], output="screen"),
        ExecuteProcess(cmd=["python3", os.path.join(HERE, "robot_markers.py")],
                       output="screen"),

        Node(package="rviz2", executable="rviz2", name="rviz2",
             arguments=["-d", os.path.join(HERE, "libi.rviz")], output="screen"),
    ])
