from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# 터틀봇 1개만 소환하고 그 터틀봇 1의 데이터를 암호화 통신 연산 복호화 하는 예제

def generate_launch_description():
    return LaunchDescription([
        # 터틀심 시뮬레이터 노드
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        
        # 암호화된 Plant 노드 (새로운 기능)
        Node(
            package='enc_turtle_cpp',
            executable='enc_turtle_plant',
            name='enc_turtle_plant',
            output='screen'
        ),
        
        # 암호화된 Controller 노드 (새로운 기능)
        Node(
            package='enc_turtle_cpp',
            executable='enc_turtle_controller',
            name='enc_turtle_controller',
            output='screen'
        ),

        # 터틀1 제어를 위한 키보드 텔레옵 노드
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -e',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel'),
            ],
            output='screen'
        )
    ]) 