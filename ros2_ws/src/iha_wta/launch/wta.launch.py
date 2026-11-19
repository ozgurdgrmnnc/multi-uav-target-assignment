"""Tüm ROS 2 node'larını başlatır (PX4 ve Gazebo ayrıca, start_sim.sh ile).

  ros2 launch iha_wta wta.launch.py num_uavs:=3 num_targets:=3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    n_uav = int(LaunchConfiguration("num_uavs").perform(context))
    n_tgt = int(LaunchConfiguration("num_targets").perform(context))
    step = float(LaunchConfiguration("step_period").perform(context))
    # use_sim_time: node'lar Gazebo saatini (/clock) kullanır. Gazebo gerçek
    # zamandan yavaş koşarsa hedefler ve İHA'lar aynı oranda yavaşlar.
    common = {"use_sim_time": True, "step_period": step}

    out = [
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen"),
        Node(package="iha_wta", executable="world_node", output="screen",
             parameters=[{**common, "num_uavs": n_uav, "num_targets": n_tgt}]),
        Node(package="iha_wta", executable="gz_viz_node", output="screen",
             parameters=[{"use_sim_time": True, "num_uavs": n_uav}]),
    ]
    for i in range(n_uav):
        out.append(Node(package="iha_wta", executable="uav_agent_node", name=f"uav_agent_{i}",
                        output="screen", parameters=[{**common, "uav_id": i}]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_uavs", default_value="3"),
        DeclareLaunchArgument("num_targets", default_value="3"),
        DeclareLaunchArgument("step_period", default_value="1.0"),
        OpaqueFunction(function=nodes),
    ])
