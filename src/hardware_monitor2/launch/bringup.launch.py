# hardware_monitor2/launch/bringup.launch.py
#
# 이동체 전체 실행용 통합 launch.
# 실행하면 이동체 id / relay 주소 / bag 여부를 터미널에서 입력받는다.
#
#   ros2 launch hardware_monitor2 bringup.launch.py
#     이동체 id [default]: car_1
#     relay 서버 주소 [ws://203.250.32.54:8080]: (엔터=기본값)
#     bag 데이터 여부 (y/t/n/f) [n]: y
#
# hardware_monitor2(상태 모니터 + rosbag 로깅 서버 + rosbridge/tf/rosapi)와
# relay_bridge(중계서버 연결)를 함께 띄운다.

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

DEFAULT_RELAY_URL = "203.250.35.87:30808"


def _truthy(value):
    # y, yes, t, true, 1 → True / 그 외(n, f, no, false...) → False
    return str(value).strip().lower() in ("y", "yes", "t", "true", "1")


def _ask(prompt, default=""):
    try:
        answer = input(prompt).strip()
    except EOFError:
        answer = ""
    return answer or default


def _normalize_ws(url):
    # ip:port 만 입력해도 ws:// 를 자동으로 붙인다.
    url = url.strip()
    if not url:
        return url
    if url.startswith("ws://") or url.startswith("wss://"):
        return url
    return "ws://" + url


def launch_setup(context, *args, **kwargs):
    print("아래의 값을 입력해주세요. Enter 입력 시 []안의 값으로 반영")
    vehicle_id = _ask("ID [default]: ", "default")
    relay_url = _normalize_ws(_ask(f"relay address [{DEFAULT_RELAY_URL}]: ", DEFAULT_RELAY_URL))
    is_bag = _truthy(_ask("bag data (y/t/n/f) [n]: ", "n"))

    print(f"[bringup] vehicle_id={vehicle_id}, relay={relay_url}, is_bag={is_bag}")

    return [
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
                'relay_server_url': relay_url,
                'is_bag': is_bag,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
