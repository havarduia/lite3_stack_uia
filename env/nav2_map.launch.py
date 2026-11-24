"""Perception's half of Nav2 on a floor map: the map, the costmaps and the planner.

    ros2 launch env/nav2_map.launch.py params:=/tmp/nav2_map.yaml

The Orin's Nav2 plans and drives (robot/orin_nav.py passes the goals). Here only the map, the
costmaps and the planner run, for the panel, cost_at, turn_blocked and route_to: planner_server
and controller_server (which holds the local costmap and its camera layer, sent on to the Orin)
without bt_navigator. The controller gets no goal; its cmd_vel goes to /rpp/cmd_vel all the same,
so nothing it says can reach the robot. map -> odom comes from lio_relay.py (the Orin's 3D fixes).

Started by env/start_nav2_map.sh, which makes that parameter file first (env/nav2_map_params.py).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

TF = [('/tf', 'tf'), ('/tf_static', 'tf_static')]      # as nav2_bringup's own launch files do
# base_link -> camera_link, the vendor's mount (dr_nav2_mapless.launch.py, which says why the pitch matters)
CAMERA_TF = ['0.25489', '0', '0.07249', '0', '0.34907', '0', 'base_link', 'camera_link']


def generate_launch_description():
    params = LaunchConfiguration('params')
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        # every node started below; not scoped, so it holds there
        SetRemap('cmd_vel', '/rpp/cmd_vel'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base2camera_tf_broadcaster',
             arguments=CAMERA_TF),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['planner_server', 'controller_server']}]),
    ])
