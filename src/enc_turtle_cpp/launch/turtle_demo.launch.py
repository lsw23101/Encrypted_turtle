from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # turtlesim 노드 시작
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # 두 번째 터틀 생성
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
                 '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}'],
            output='screen'
        ),
        
        # Plant 노드 시작
        Node(
            package='enc_turtle_cpp',
            executable='turtle_plant',
            name='turtle_plant'
        ),
        
        # Controller 노드 시작
        Node(
            package='enc_turtle_cpp',
            executable='turtle_controller',
            name='turtle_controller'
        ),
        
        # 암호화된 Plant 노드 시작
        Node(
            package='enc_turtle_cpp',
            executable='enc_turtle_plant',
            name='enc_turtle_plant'
        ),
        
        # 암호화된 Controller 노드 시작
        Node(
            package='enc_turtle_cpp',
            executable='enc_turtle_controller',
            name='enc_turtle_controller'
        ),
        
        # Teleop 노드 시작 (터틀1 제어용)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel'),
            ]
        )
    ]) 