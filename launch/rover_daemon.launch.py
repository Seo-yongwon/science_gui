"""로버(UMPC)만 실행: rover.launch 전체가 꺼져 있을 때 켜 두는 최소 데몬.

사용 순서:
  1) 로버에서 이 파일만 실행한다.
       ros2 launch science_gui rover_daemon.launch.py
  2) 기지국에서 mission_gui 의 「로버 스택」 버튼으로 rover.launch.py 시작.

동일 머신에서 rover.launch 가 이미 돌고 있다면 이 데몬은 필요 없다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace_setup',
            default_value='~/science_ws/install/setup.bash',
        ),
        DeclareLaunchArgument(
            'launch_extra_args',
            default_value='',
            description='rover.launch 에 넘길 추가 인자 (공백 구분)',
        ),
        Node(
            package='science_gui',
            executable='rover_launch_trigger',
            name='rover_launch_trigger',
            output='screen',
            parameters=[{
                'workspace_setup': LaunchConfiguration('workspace_setup'),
                'launch_extra_args': LaunchConfiguration('launch_extra_args'),
            }],
        ),
    ])
