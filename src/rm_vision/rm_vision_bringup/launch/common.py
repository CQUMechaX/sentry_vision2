import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node

leftlaunch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'leftlaunch_params.yaml')))

rightlaunch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'rightlaunch_params.yaml')))

leftrobot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_leftgimbal.urdf.xacro'),
    ' xyz:=', leftlaunch_params['leftodom2leftcamera']['xyz'], ' rpy:=', leftlaunch_params['leftodom2leftcamera']['rpy']])

rightrobot_description = Command(['xacro ', os.path.join(
    get_package_share_directory('rm_gimbal_description'), 'urdf', 'rm_rightgimbal.urdf.xacro'),
    ' xyz:=', rightlaunch_params['rightodom2rightcamera']['xyz'], ' rpy:=', rightlaunch_params['rightodom2rightcamera']['rpy']])

# 可视化
# 将机器人的URDF描述转换为实际的机器人状态信息，并以指定的频率发布给其他节点和可视化工具
leftrobot_state_publisher = Node(
    # ROS标准包，用于发布机器人的状态信息
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': leftrobot_description,
                 'publish_frequency': 1000.0}] # 机器人状态发布的频率
)

rightrobot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': rightrobot_description,
                 'publish_frequency': 1000.0}]
)

leftnode_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'leftnode_params.yaml')

rightnode_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'rightnode_params.yaml')

lefttracker_node = Node(
    package='leftarmor_tracker',
    executable='leftarmor_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[leftnode_params],
    ros_arguments=['--log-level', 'leftarmor_tracker:='+leftlaunch_params['tracker_log_level']],
)

righttracker_node = Node(
    package='rightarmor_tracker',
    executable='rightarmor_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[rightnode_params],
    ros_arguments=['--log-level', 'rightarmor_tracker:='+rightlaunch_params['tracker_log_level']],
)
