# hardware_monitor2/launch/bringup.launch.py
#
# 이동체 전체 실행용 통합 launch.
# hardware_monitor2(상태 모니터 + rosbag 로깅 서버 + rosbridge/tf/rosapi)와
# relay_bridge(중계서버 연결)를 한 번에 띄운다.
#
# 사용 예:
#   ros2 launch hardware_monitor2 bringup.launch.py \
#       vehicle_id:=car_1 relay_server_url:=ws://203.250.32.54:8080
#
# 개별 실행이 필요하면 기존 launch.py(모니터만) 또는
# `ros2 run relay_bridge relay_bridge_node`를 그대로 사용하면 된다.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- relay_bridge 실행 인자 ----
    vehicle_id = LaunchConfiguration('vehicle_id')
    relay_server_url = LaunchConfiguration('relay_server_url')

    declare_vehicle_id = DeclareLaunchArgument(
        'vehicle_id',
        default_value='default',
        description='중계서버에 등록할 차량 ID',
    )
    declare_relay_server_url = DeclareLaunchArgument(
        'relay_server_url',
        default_value='ws://localhost:8080',
        description='중계서버 WebSocket 주소',
    )

    return LaunchDescription([
        # ---- 실행 인자 선언 ----
        declare_vehicle_id,
        declare_relay_server_url,

        # ---- hardware_monitor2: 상태 모니터 ----
        Node(
            package='hardware_monitor2',
            executable='send_hunter',
            name='send_hunter',
            output='screen',
        ),

        # ---- hardware_monitor2: rosbag 로깅 서비스 서버(/logging) ----
        Node(
            package='hardware_monitor2',
            executable='add_two_ints',
            name='add_two_ints',
            output='screen',
            # respawn=True,
        ),

        # ---- rosbridge 계열(브라우저 ↔ ROS) ----
        Node(
            package='tf2_web_republisher_py',
            executable='tf2_web_republisher',
            name='tf2_web_republisher',
            output='screen',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen',
        ),

        # ---- relay_bridge: 중계서버 연결(송신) ----
        Node(
            package='relay_bridge',
            executable='relay_bridge_node',
            name='relay_bridge_node',
            output='screen',
            parameters=[{
                'vehicle_id': vehicle_id,
                'relay_server_url': relay_server_url,
            }],
        ),
    ])
