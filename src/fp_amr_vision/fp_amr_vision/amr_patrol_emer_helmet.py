#!/usr/bin/env python3
import json
import math
import time

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseStamped, Quaternion


ROBOT_NAMESPACE = 'robot6'  # 실제 로봇 네임스페이스로 변경 (ros2 topic list로 확인)

# fleet_fsm 과의 명령/상태 토픽용 QoS (fleet_fsm.py 의 COMMAND_QOS 와 동일해야
# 매칭됨): RELIABLE 재전송 + TRANSIENT_LOCAL(latched) 로 discovery 경합 시에도
# 마지막 명령 1개가 유실되지 않게 한다. 터미널 테스트 시에도 QoS 를 맞춰야 한다:
#   ros2 topic pub -1 --qos-reliability reliable \
#     --qos-durability transient_local /robot6/emergency_clear \
#     std_msgs/msg/Bool "{data: true}"
COMMAND_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
BATTERY_LOW_THRESHOLD = 0.3  # 30% 미만이면 순찰 중단하고 도킹 복귀

DOCK_POSITION = [3.8, 6.06]
DOCK_DIRECTION = TurtleBot4Directions.NORTH

# === fleet_fsm 연동 개요 ===
# 이 노드는 로봇 선정 판단을 하지 않는 "실행부"다. fleet_fsm(상태머신)이
#  - 순찰: 배터리가 가장 높은 idle 로봇 선정
#  - 응급/안전모: 목표 지점에서 가장 가까운 로봇 선정
# 을 담당하고, 선정된 로봇의 전용 토픽(f'/{robot_id}/...')으로만 명령을
# 발행한다. 선정에 필요한 정보는 fleet_fsm 이 각 로봇 네임스페이스의
#  - /robotX/battery_state (sensor_msgs/BatteryState): 배터리 잔량
#  - /robotX/amcl_pose (geometry_msgs/PoseWithCovarianceStamped): 현재 위치
#  - /robotX/amr_status (std_msgs/String, 아래 STATUS_TOPIC): IDLE/PATROL/... 상태
# 를 구독해서 얻는다.

# fleet_fsm 이 순찰을 지시하는 토픽. 노드는 시작 후 도킹 상태로 대기하다가
# 이 명령을 받아야만 순찰을 시작한다. payload 는 JSON 문자열이며 robot_id 는
# 선택(실려 오면 자기 것인지 검증). 터미널 테스트:
#   ros2 topic pub -1 /robot6/patrol_cmd std_msgs/msg/String \
#     '{data: "{\"robot_id\": \"robot6\"}"}'
PATROL_CMD_TOPIC = 'patrol_cmd'

# 이 노드가 fleet_fsm 에 현재 상태를 알리는 토픽. 로봇 선정 시
# "idle 인 로봇 중에서" 를 판단하는 근거가 된다.
# 발행 값: IDLE / PATROL / EMERGENCY / HELMET / RETURNING
STATUS_TOPIC = 'amr_status'

# fleet_fsm 이 응급 상황 발생 시 로봇을 선정한 뒤 좌표만 발행하는 로봇 전용
# 토픽. 도킹 대기 중이든 순찰 중이든 이 토픽으로 목표가 들어오면 즉시
# undock 후 이동한다 (실제 로봇 선정/우선순위 판단은 fleet_fsm 이 담당하고,
# 이 노드는 좌표를 받아 이동만 수행하는 실행부).
EMERGENCY_GOAL_TOPIC = 'emergency_goal'

# 실습/디버그용: 응급 상황 종료(조치 완료)를 수동으로 알리는 토픽. 실제
# 연동 전까지는 터미널에서 아래처럼 흉내낼 수 있다:
#   ros2 topic pub -1 /robot6/emergency_clear std_msgs/msg/Bool "{data: true}"
# 목표 지점 도착 후에는 이 신호를 받을 때까지 정해진 시간 없이 그 자리에서
# 무기한 대기한다 (충분한 응급조치가 이루어질 때까지). 이동 중에 받으면
# 즉시 취소하고 그 자리에서 대기 상태로 전환한다.
EMERGENCY_CLEAR_TOPIC = 'emergency_clear'

# 응급/순찰보다 낮은 우선순위: 도킹/idle 상태일 때만 안전모 배달 지점으로
# 이동한다. fleet_fsm 이 가장 가까운 idle 로봇을 선정해 발행하며, robot_id 는
# 선택(실려 오면 자기 것인지 검증). 실제 연동 전 터미널 테스트:
#   ros2 topic pub -1 /robot6/helmet_goal std_msgs/msg/String \
#     '{data: "{\"x\": 1.0, \"y\": 2.0, \"robot_id\": \"robot6\"}"}'
HELMET_GOAL_TOPIC = 'helmet_goal'
# 순찰이 끝나고 도킹/idle 상태가 된 뒤, 안전모 배달을 실제로 시작하기까지
# 기다리는 시간(초). 막 도킹 완료한 로봇이 곧바로 재출동하지 않도록 하는
# 유예 시간으로, 필요에 맞게 조정한다.
HELMET_IDLE_DELAY = 3.0
HELMET_DELIVERY_HOLD = 5.0  # 배달 지점 도착 후 현장 대기(초)

# dock() 전에는 도킹 스테이션 좌표(DOCK_POSITION)로 바로 가지 않고
# 이 여유 지점까지만 이동한다. DOCK_POSITION은 코스트맵상 도킹 스테이션
# 본체와 겹쳐 있어 곧장 goToPose하면 경로 계획이 실패할 수 있고,
# 마지막 정밀 접속은 IR 센서 기반의 dock()에 맡겨야 하기 때문이다.
DOCK_APPROACH_POSITION = [3.8, 6.06]

# === 순찰 웨이포인트 (RViz에서 클릭해서 얻은 지점) ===
# (x, y) 형태면 다음 지점을 바라보는 방향으로 yaw가 자동 계산되는 일반 순찰 지점이고,
# (x, y, yaw_deg) 형태면 소화기 포인트로 간주해 그 yaw를 바라보고 도착한 뒤
# ARUCO_CHECK_TOPIC에서 대조 완료 신호를 받을 때까지 대기했다가 다음 지점으로 이동한다.
PATROL_POINTS = [
    (-2.5822043418884277, -0.017423417419195175),
    (-2.3063042163848877, 1.8556833267211914),
    (0.32103753089904785, 2.54193377494812, 180.0),  # 소화기 포인트 1
    (3.7301056385040283, 3.468379497528076, 0.0),  # 소화기 포인트 2
    # (3.8219058513641357, 6.112329006195068),
    (3.1837000846862793, 1.4198600053787231),
    (0.5075345635414124, 2.354707717895508),
]

# 소화기 포인트에서 아루코 마커 인식/대조가 끝났는지 알려주는 디버그용 토픽.
# 실제 인식 노드가 준비되기 전까지는 터미널에서 아래처럼 흉내낼 수 있다:
#   ros2 topic pub -1 /robot2/aruco_check_done std_msgs/msg/Bool "{data: true}"
ARUCO_CHECK_TOPIC = 'aruco_check_done'


def get_pose_with_yaw(navigator, x, y, yaw_deg):
    """TurtleBot4Directions(4방향)로는 표현 안 되는 임의의 yaw(도 단위)로 PoseStamped 생성"""
    yaw = math.radians(yaw_deg)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation = Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )
    return pose


class PatrolNavigator(TurtleBot4Navigator):
    """순찰 중에도 배터리 상태를 계속 구독하는 네비게이터."""

    def __init__(self):
        super().__init__()
        self.battery_percentage = None
        self.aruco_check_done = False
        # fleet_fsm 이 로봇을 선정한 뒤 내려보내는 응급 출동 좌표. 콜백과
        # 메인 루프가 동일 스레드(rclpy.spin_once 기반)에서 돌기 때문에
        # 별도 락 없이도 안전하다 (배터리/아루코 상태와 동일한 방식).
        self.pending_emergency = None
        self.emergency_clear_requested = False
        self.pending_helmet = None
        self.patrol_requested = False

        self.status_pub = self.create_publisher(String, STATUS_TOPIC, COMMAND_QOS)

        self.create_subscription(
            BatteryState,
            'battery_state',
            self._battery_callback,
            10,
        )

        self.create_subscription(
            Bool,
            ARUCO_CHECK_TOPIC,
            self._aruco_check_callback,
            10,
        )

        self.create_subscription(
            String,
            EMERGENCY_GOAL_TOPIC,
            self._emergency_callback,
            COMMAND_QOS,
        )

        self.create_subscription(
            Bool,
            EMERGENCY_CLEAR_TOPIC,
            self._emergency_clear_callback,
            COMMAND_QOS,
        )

        self.create_subscription(
            String,
            HELMET_GOAL_TOPIC,
            self._helmet_callback,
            COMMAND_QOS,
        )

        self.create_subscription(
            String,
            PATROL_CMD_TOPIC,
            self._patrol_cmd_callback,
            COMMAND_QOS,
        )

    def publish_status(self, state: str):
        msg = String()
        msg.data = state
        self.status_pub.publish(msg)
        self.info(f'상태 변경: {state}')

    def _battery_callback(self, msg: BatteryState):
        self.battery_percentage = msg.percentage

    def _aruco_check_callback(self, msg: Bool):
        if msg.data:
            self.aruco_check_done = True

    def _emergency_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            item = {
                'x': float(data['x']),
                'y': float(data['y']),
                'reason': data.get('reason', 'EMERGENCY'),
            }
        except Exception as e:
            self.error(f'Invalid emergency_goal message: {e}')
            return
        # fleet_fsm 이 로봇 전용 토픽(f'/{robot_id}/emergency_goal')으로만
        # 발행하므로 원래 토픽 이름 자체가 이미 자기 id 로 스코프돼 있지만,
        # payload 에 robot_id 가 실려 오는 경우 한 번 더 검증해 다른 로봇 앞으로
        # 온 메시지를 잘못 수신(오배선/디버그 pub 실수 등)했을 때 무시한다.
        target = data.get('robot_id')
        if target and target != ROBOT_NAMESPACE:
            self.info(f'다른 로봇({target}) 대상 응급 신호 - 무시합니다.')
            return
        # 최신 목표 1개만 유지 - 순찰 중 이동/대조 대기 도중에 와도 즉시
        # 눈에 띄어야 하므로 이전에 처리 못한 목표가 있어도 덮어쓴다.
        self.pending_emergency = item
        self.info(
            f"응급 출동 신호 수신 (reason={item['reason']}): "
            f"({item['x']:.2f}, {item['y']:.2f})"
        )

    def _emergency_clear_callback(self, msg: Bool):
        if msg.data:
            self.emergency_clear_requested = True
            self.info('응급 상황 종료 신호 수신.')

    def _helmet_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            item = {'x': float(data['x']), 'y': float(data['y'])}
        except Exception as e:
            self.error(f'Invalid helmet_goal message: {e}')
            return
        target = data.get('robot_id')
        if target and target != ROBOT_NAMESPACE:
            self.info(f'다른 로봇({target}) 대상 안전모 신호 - 무시합니다.')
            return
        self.pending_helmet = item
        self.info(f"안전모 배달 신호 수신: ({item['x']:.2f}, {item['y']:.2f})")

    def _patrol_cmd_callback(self, msg: String):
        # payload 없이 빈 문자열로 와도 순찰 시작으로 처리한다.
        data = {}
        if msg.data.strip():
            try:
                data = json.loads(msg.data)
            except Exception as e:
                self.error(f'Invalid patrol_cmd message: {e}')
                return
        target = data.get('robot_id')
        if target and target != ROBOT_NAMESPACE:
            self.info(f'다른 로봇({target}) 대상 순찰 명령 - 무시합니다.')
            return
        self.patrol_requested = True
        self.info('순찰 시작 명령 수신.')

    def take_patrol_cmd(self):
        requested = self.patrol_requested
        self.patrol_requested = False
        return requested

    def has_pending_emergency(self):
        return self.pending_emergency is not None

    def take_emergency(self):
        item = self.pending_emergency
        self.pending_emergency = None
        return item

    def take_emergency_clear(self):
        cleared = self.emergency_clear_requested
        self.emergency_clear_requested = False
        return cleared

    def has_pending_helmet(self):
        return self.pending_helmet is not None

    def take_helmet(self):
        item = self.pending_helmet
        self.pending_helmet = None
        return item

    def is_battery_low(self):
        return (
            self.battery_percentage is not None
            and self.battery_percentage < BATTERY_LOW_THRESHOLD
        )


def build_patrol_waypoints(navigator):
    """순찰 지점들을 (PoseStamped, wait_for_aruco) 튜플 리스트로 변환."""
    goal_pose = []

    for i, point in enumerate(PATROL_POINTS):
        x, y = point[0], point[1]
        # yaw가 지정된 지점(3-튜플)은 소화기 포인트로 간주해 대조 완료를 기다린다.
        wait_for_aruco = len(point) >= 3 # true / false

        if wait_for_aruco:
            yaw_deg = point[2]
        else:
            # yaw가 없으면 다음 지점을 바라보며 도착하도록 자동 계산한다.
            if i < len(PATROL_POINTS) - 1:
                nx, ny = PATROL_POINTS[i + 1][0], PATROL_POINTS[i + 1][1]
            else:
                nx, ny = DOCK_POSITION  # 마지막 순찰 지점은 복귀 지점 방향을 봄
            yaw_deg = math.degrees(math.atan2(ny - y, nx - x))

        pose = get_pose_with_yaw(navigator, x, y, yaw_deg)
        goal_pose.append((pose, wait_for_aruco))

    goal_pose.append((navigator.getPoseStamped(DOCK_POSITION, DOCK_DIRECTION), False))
    return goal_pose


def _cancel_and_wait(navigator):
    """ cancelTask()는 취소 "요청"이 접수됐는지만 확인하고 반환하므로,
    이전 작업이 서버에서 완전히 종료될 때까지 기다린 뒤 다음 goToPose를
    보내야 한다. 그렇지 않으면 새 goToPose가 이전 취소 상태와 뒤섞여
    이동도 하기 전에 즉시 완료된 것으로 오판될 수 있다. """
    navigator.cancelTask()
    while not navigator.isTaskComplete():
        pass


def run_patrol(navigator, waypoints, start_index=0):
    """
    웨이포인트를 하나씩 goToPose로 순찰한다. start_index 부터 시작하므로
    응급 출동으로 중단됐던 지점을 그대로 이어서 재개할 수 있다.

    소화기 포인트(wait_for_aruco=True)에 도착하면 ARUCO_CHECK_TOPIC으로
    대조 완료 신호가 올 때까지 대기했다가 다음 지점으로 이동한다.

    배터리가 낮아지거나 응급 출동 신호가 들어오면 즉시 태스크를 취소하고
    (result, 중단된 waypoint index) 를 반환한다 ('battery_low' / 'emergency').
    끝까지 완주하면 ('completed', None).
    """
    for idx in range(start_index, len(waypoints)):
        pose, wait_for_aruco = waypoints[idx]
        navigator.goToPose(pose)

        while not navigator.isTaskComplete():
            if navigator.has_pending_emergency():
                navigator.info('응급 출동 신호 감지 - 순찰을 중단합니다.')
                _cancel_and_wait(navigator)
                return 'emergency', idx
            if navigator.is_battery_low():
                navigator.info(
                    f'배터리 잔량 {navigator.battery_percentage * 100:.1f}% '
                    f'({BATTERY_LOW_THRESHOLD * 100:.0f}% 미만) - 순찰을 중단하고 복귀합니다.'
                )
                _cancel_and_wait(navigator)
                return 'battery_low', idx

        if wait_for_aruco:
            navigator.info('소화기 포인트 도착. 아루코 마커 대조 결과를 기다립니다...')
            navigator.aruco_check_done = False
            while not navigator.aruco_check_done:
                rclpy.spin_once(navigator, timeout_sec=0.1)
                if navigator.has_pending_emergency():
                    navigator.info('응급 출동 신호 감지 - 순찰을 중단합니다.')
                    return 'emergency', idx
                if navigator.is_battery_low():
                    navigator.info(
                        f'배터리 잔량 {navigator.battery_percentage * 100:.1f}% '
                        f'({BATTERY_LOW_THRESHOLD * 100:.0f}% 미만) - 순찰을 중단하고 복귀합니다.'
                    )
                    return 'battery_low', idx
            navigator.info('아루코 마커 대조 완료. 다음 지점으로 이동합니다.')

    return 'completed', None


def run_emergency_dispatch(navigator):
    """
    대기 중인 응급 출동 좌표로 이동한다. 기존 순찰 웨이포인트 이동과 동일한
    get_pose_with_yaw + goToPose 로직을 재사용하되, yaw 는 지정되지 않으므로
    0.0 으로 둔다. 도킹 중이면 먼저 undock 하고, 이동 중 더 최신 응급 신호가
    들어오면 현재 이동을 취소하고 새 목표로 즉시 갱신한다.

    목표 지점 도착 후에는 emergency_clear(디버그용 조치 완료 신호)를 받을
    때까지 정해진 시간 없이 그 자리에서 무기한 대기한다 - 실제 응급조치가
    끝나는 시점은 현장 상황에 따라 다르므로 fixed hold 로 임의 종료하지
    않는다. 대기 중 더 최신 응급 신호가 오면 그 목표로 즉시 갱신한다.
    """
    # 이전 응급 상황에서 온(또는 latched 로 남아 있던) clear 신호가 새 응급의
    # 대기를 즉시 끝내버리지 않도록, 출동 시작 시점에 잔여 플래그를 버린다.
    navigator.take_emergency_clear()

    while navigator.has_pending_emergency():
        item = navigator.take_emergency()
        x, y, reason = item['x'], item['y'], item['reason']
        navigator.info(f'응급 출동 시작(reason={reason}): ({x:.2f}, {y:.2f})')

        if navigator.getDockedStatus():
            navigator.info('도킹 상태 - undock 후 출동합니다.')
            navigator.undock()

        pose = get_pose_with_yaw(navigator, x, y, 0.0)
        navigator.goToPose(pose)

        while not navigator.isTaskComplete():
            if navigator.take_emergency_clear():
                navigator.info('이동 중 상황 종료 신호 수신 - 출동을 중단합니다.')
                _cancel_and_wait(navigator)
                return
            if navigator.has_pending_emergency():
                navigator.info('더 최신 응급 신호 수신 - 목표를 갱신합니다.')
                _cancel_and_wait(navigator)
                break
        else:
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                navigator.info(
                    f'응급 출동 지점 도착: ({x:.2f}, {y:.2f}). '
                    '조치 완료(emergency_clear) 신호를 기다립니다.'
                )
            else:
                navigator.error(
                    f'응급 출동 이동 실패 (result={result}). '
                    '그 자리에서 조치 완료 신호를 기다립니다.'
                )

            # 응급조치가 충분히 이루어질 때까지 그 자리에서 무기한 대기한다.
            while not navigator.take_emergency_clear():
                if navigator.has_pending_emergency():
                    break
                rclpy.spin_once(navigator, timeout_sec=0.1)
            else:
                navigator.info('응급 상황 종료 신호 수신 - 순찰을 이어서 진행합니다.')
                return
            # break 로 빠져나온 경우(더 최신 응급 신호) 바깥 while 이 이어서 처리


def run_helmet_delivery(navigator):
    """
    응급/순찰보다 낮은 우선순위로 안전모 배달 지점까지 이동한다. 도킹/idle
    상태(main() 의 대기 루프)에서만 호출되므로 순찰이나 응급 출동을 방해하지
    않는다. 이동 중 응급 신호가 들어오면 즉시 중단하고 응급 대응에 양보한다.
    """
    while navigator.has_pending_helmet():
        if navigator.has_pending_emergency():
            navigator.info('응급 신호 우선 - 안전모 배달을 시작하지 않습니다.')
            return

        item = navigator.take_helmet()
        x, y = item['x'], item['y']
        navigator.info(f'안전모 배달 시작: ({x:.2f}, {y:.2f})')

        if navigator.getDockedStatus():
            navigator.undock()

        pose = get_pose_with_yaw(navigator, x, y, 0.0)
        navigator.goToPose(pose)

        while not navigator.isTaskComplete():
            if navigator.has_pending_emergency():
                navigator.info('안전모 배달 중 응급 신호 수신 - 중단합니다.')
                _cancel_and_wait(navigator)
                return
        else:
            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                navigator.info(f'안전모 배달 지점 도착: ({x:.2f}, {y:.2f})')
            else:
                navigator.error(f'안전모 배달 이동 실패 (result={result})')

            deadline = time.monotonic() + HELMET_DELIVERY_HOLD
            while time.monotonic() < deadline:
                rclpy.spin_once(navigator, timeout_sec=0.1)
                if navigator.has_pending_emergency():
                    break

    navigator.info('안전모 배달 완료. 도킹 스테이션으로 복귀합니다.')
    _return_to_dock(navigator)


def main():
    rclpy.init(args=[
        '--ros-args',
        '-r', f'__ns:=/{ROBOT_NAMESPACE}',
        '-r', f'/tf:=/{ROBOT_NAMESPACE}/tf',
        '-r', f'/tf_static:=/{ROBOT_NAMESPACE}/tf_static',
    ])

    navigator = PatrolNavigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped(DOCK_POSITION, DOCK_DIRECTION)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    waypoints = build_patrol_waypoints(navigator)

    navigator.info('fleet_fsm 명령 대기를 시작합니다 (patrol_cmd / emergency_goal / helmet_goal).')

    # 응급 출동으로 중단된 순찰 웨이포인트 index. 응급 조치가 끝나면 처음부터
    # 다시 돌지 않고 이 지점부터 이어서 순찰한다.
    patrol_index = 0
    # 순찰은 fleet_fsm 의 patrol_cmd 를 받아야만 시작한다. 응급 출동으로
    # 중단된 경우에도 이 플래그가 살아 있으므로 조치 후 이어서 순찰한다.
    patrol_active = False

    navigator.publish_status('IDLE')
    idle_since = time.monotonic()

    # 메인 루프: 우선순위는 응급 > 순찰(명령 기반) > 안전모 배달.
    # 도킹 대기 중이든 순찰 중이든 emergency_goal 이 오면 최우선으로 반응한다.
    while rclpy.ok():
        if navigator.has_pending_emergency():
            navigator.publish_status('EMERGENCY')
            run_emergency_dispatch(navigator)
            if not patrol_active:
                # 재개할 순찰이 없으면 도킹 스테이션으로 복귀해 대기한다.
                navigator.publish_status('RETURNING')
                _return_to_dock(navigator)
                navigator.publish_status('IDLE')
                idle_since = time.monotonic()
            continue

        if navigator.take_patrol_cmd():
            patrol_active = True
            patrol_index = 0

        if patrol_active:
            if navigator.getDockedStatus():
                navigator.undock()
            navigator.publish_status('PATROL')
            result, idx = run_patrol(navigator, waypoints, start_index=patrol_index)

            if result == 'emergency':
                # 응급 신호는 run_patrol 이 이미 취소만 해두고 반환했으므로,
                # 다음 루프 최상단에서 run_emergency_dispatch 로 처리된다.
                # 중단된 지점을 기억해뒀다가 응급 조치가 끝나면 이어서 순찰한다.
                patrol_index = idx
                continue

            if result == 'battery_low':
                navigator.info('배터리 부족으로 순찰을 중단했습니다. 도킹 스테이션으로 복귀합니다.')
            else:
                navigator.info('순찰 완료. 도킹 스테이션으로 복귀합니다.')
            patrol_active = False
            patrol_index = 0
            navigator.publish_status('RETURNING')
            _return_to_dock(navigator)
            navigator.publish_status('IDLE')
            navigator.info('대기 모드: 도킹 상태에서 다음 명령을 기다립니다.')
            idle_since = time.monotonic()
            continue

        # 대기 모드: 도킹 상태에서 fleet_fsm 명령을 기다린다 (spin_once 로만
        # 대기하므로 CPU 를 거의 쓰지 않는다). 안전모 배달은 HELMET_IDLE_DELAY
        # 초 동안 idle 상태가 유지된 뒤에만 시작한다 (막 도킹한 로봇이 곧바로
        # 재출동하지 않도록 하는 유예 시간).
        rclpy.spin_once(navigator, timeout_sec=0.2)
        if (navigator.has_pending_helmet()
                and time.monotonic() - idle_since >= HELMET_IDLE_DELAY):
            navigator.publish_status('HELMET')
            run_helmet_delivery(navigator)
            if not navigator.has_pending_emergency():
                navigator.publish_status('IDLE')
            idle_since = time.monotonic()

    rclpy.shutdown()


def _return_to_dock(navigator):
    """ 순찰 중단 지점은 도킹 스테이션과 멀 수 있으므로, dock()을 바로 부르지
    않고 도킹 스테이션 앞 여유 지점까지 먼저 이동한 뒤 dock() 한다. """
    dock_approach_pose = navigator.getPoseStamped(DOCK_APPROACH_POSITION, DOCK_DIRECTION)
    navigator.goToPose(dock_approach_pose)
    while not navigator.isTaskComplete():
        pass

    approach_result = navigator.getResult()
    if approach_result != TaskResult.SUCCEEDED:
        navigator.error(
            f'도킹 스테이션 앞 지점 이동 실패 (result={approach_result}). '
            '현재 위치에서 바로 도킹을 시도합니다.'
        )

    navigator.dock()


if __name__ == '__main__':
    main()
