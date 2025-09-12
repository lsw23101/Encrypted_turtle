from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# 리더 - 팔로워 예제에서 터틀봇 1의 pose를 암호화 통신, 연산하는 예제

def generate_launch_description():
    return LaunchDescription([
        # 터틀심 시뮬레이터 노드
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # 두 번째 터틀 생성
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
                 '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}'],
            output='screen'
        ),
                
        # 컨트롤러 노드 먼저 실행하여 암호화 cc eval 준비
        # 암호화된 Controller 노드 (새로운 기능)
        Node(
            package='enc_turtle_cpp',
            executable='test_controller',
            name='test_controller',
            output='screen'
        ),

        # 암호화된 Plant 노드 (새로운 기능)
        Node(
            package='enc_turtle_cpp',
            executable='test_plant',
            name='test_plant',
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