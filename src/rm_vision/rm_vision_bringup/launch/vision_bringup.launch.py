import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import leftnode_params,rightnode_params, leftlaunch_params,rightlaunch_params, leftrobot_state_publisher,rightrobot_state_publisher, lefttracker_node, righttracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,    # 软件包
            plugin=plugin,      # 插件，相当于调用其中的哪个类
            name='camera_node', # 节点名称
            parameters=[leftnode_params, rightnode_params], # 参数列表，保存在config中
            #启用了节点间的进程内通信，这意味着节点之间可以更高效地进行通信
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='leftcamera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='leftarmor_detector',
                    plugin='leftrm_auto_aim::ArmorDetectorNode',
                    name='leftarmor_detector',
                    parameters=[leftnode_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='rightarmor_tracker',
                    plugin='rightrm_auto_aim::ArmorTrackerNode',
                    name='rightarmor_tracker',
                    parameters=[rightnode_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'leftarmor_detector:='+leftlaunch_params['detector_log_level'],
                           'rightarmor_detector:='+rightlaunch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')

    if (leftlaunch_params['camera'] == 'hik'):
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif (leftlaunch_params['camera'] == 'mv'):
        cam_detector = get_camera_detector_container(mv_camera_node)

    camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        name='mindvision_camera_node',
        output='both',
        emulate_tty=True,
        parameters=[leftnode_params,rightnode_params],
        on_exit=Shutdown()
    )

    serial_driver_node = Node(
        package='rm_serial_driver',
        executable='rm_serial_driver_node',
        name='serial_driver',
        output='both',
        emulate_tty=True,
        parameters=[leftnode_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'serial_driver:='+leftlaunch_params['serial_log_level']],
    )

    leftdetector_node = Node(
        package='leftarmor_detector',
        executable='leftarmor_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[leftnode_params],
        arguments=['--ros-args', '--log-level',
                   'leftarmor_detector:='+leftlaunch_params['detector_log_level']],
    )

    rightdetector_node = Node(
        package='rightarmor_detector',
        executable='rightarmor_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[rightnode_params],
        arguments=['--ros-args', '--log-level',
                   'rightarmor_detector:='+rightlaunch_params['detector_log_level']],
    )

    lefttrajectory_node = Node(
        package='mechax_trajectory',
        executable='leftmechax_trajectory',
        name='leftmechax_trajectory',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    righttrajectory_node = Node(
        package='mechax_trajectory',
        executable='rightmechax_trajectory',
        name='rightmechax_trajectory',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    # delay_serial_node 会以1.5秒的周期触发执行串口驱动节点，
    # 而 delay_tracker_node 会以2.0秒的周期触发执行追踪节点。
    delay_serial_node = TimerAction(
        period=1.5,
        actions=[serial_driver_node],
    )

    leftdelay_tracker_node = TimerAction(
        period=2.0,
        actions=[lefttracker_node],
    )

    rightdelay_tracker_node = TimerAction(
        period=2.0,
        actions=[righttracker_node],
    )

    # delay_trajectory_node = TimerAction(
    #     period=2.5,
    #     actions=[trajectory_node],
    # )

    return LaunchDescription([
        leftrobot_state_publisher, # 可视化
        rightrobot_state_publisher,
        #cam_detector,          # 相机+detector
        camera_node,
        delay_serial_node,     # 串口通信
        leftdetector_node,
        rightdetector_node,
        leftdelay_tracker_node,    # tracker
        rightdelay_tracker_node,
        lefttrajectory_node, # 轨迹规划,弹道解算
        righttrajectory_node,
    ])
