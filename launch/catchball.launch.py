import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    action_catchball = Node(
        package='action_findball',
        executable='catchball_server',
        name='catchball_server',
        output='screen', #用于将话题信息打印到屏幕
    )
    
    approaching_ball = Node(
        package='action_findball',
        executable='approaching_ball',
        name='approaching_ball',
        output='screen', #用于将话题信息打印到屏幕
    )
    
    
    findball = Node(
        package='action_findball',
        executable='publisher_findball',
        name='findball',
        output='screen', #用于将话题信息打印到屏幕
    )
    
    # pub_map2odom_cmd = ExecuteProcess(
    #     cmd = ['ros2','run','tf2_ros','static_transform_publisher','0','0','0','0','0','0','map','odom'],
    #     cwd = [pkg_share],
    #     output='screen',
    #     condition=IfCondition()
    # )

    #ld.add_action(action_catchball)
    ld.add_action(findball)
    ld.add_action(approaching_ball)

    return ld