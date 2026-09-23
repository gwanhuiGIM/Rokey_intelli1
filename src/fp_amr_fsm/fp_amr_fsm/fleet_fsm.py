#!/usr/bin/env python3
"""
Fleet FSM 관제 노드 (PC4 의 fleet_manager + mission_orchestrator + dashboard 역할).

AMR 구동 노드(patrol_fire.py)와 분리된 '판단 + 관제 표시' 전용 노드.
patrol_fire.py 는 로봇마다 독립 실행되는 standalone 스크립트로, 하드코딩된
waypoint 순찰과 배터리부족 자동 복귀를 전부 자체적으로 수행한다. 이 노드는
그 순찰 자체에는 관여하지 않고(명령 불필요), 오직 웹캠이 감지한 응급/안전모
이벤트가 들어왔을 때 어떤 로봇을 보낼지 "선정"만 하고, 선정된 로봇의 전용
토픽(f'/{robot_id}/emergency_goal')에 목표 좌표만 발행한다. patrol_fire.py
코드는 수정하지 않는다는 전제이므로, 이 노드는 patrol_fire.py 가 구독하기로
한 그 토픽 계약(스키마)만 지키면 된다.

각 로봇의 '구동 상태'(도킹/이동/대기)는 로봇의 자체 보고를 신뢰하지 않고,
로봇이 실제로 발행하는 표준 ROS 토픽을 이 노드가 직접 구독해서 판단한다
(day3/3_1_e_mail_delivery.py 가 TurtleBot4Navigator의 getDockedStatus()/
goToPose 처럼 로봇의 실제 동작 primitive 를 기준으로 삼는 것과 같은 방식).
patrol_fire.py 는 순찰/점검/배터리복귀 결과를 별도 이벤트로 보고하지 않으므로,
출동 완료 여부도 raw 토픽(Nav2 액션 상태의 navigating True->False 전이)만으로
추론한다.

    구동 상태 판단 근거 (raw 토픽, 표준 ROS 인터페이스만 사용):
      DOCKED     <- /<robot>/dock_status        (irobot_create_msgs/DockStatus.is_docked)
      NAVIGATING <- /<robot>/navigate_to_pose/_action/status (Nav2 액션 상태,
                     ACCEPTED/EXECUTING 이면 이동 중)
      IDLE       <- 위 둘 다 아닐 때
      위치       <- /<robot>/amcl_pose
      배터리     <- /<robot>/battery_state
      하트비트   <- 위 네 토픽 중 아무거나 수신되면 '살아있음'으로 간주
                     (OFFLINE_TIMEOUT 동안 전혀 없으면 OFFLINE)

================================ README ================================
1) 실행 전 준비
   - 이 노드는 patrol_fire.py 가 로봇마다 하나씩 떠서 자체 순찰 중이라고
     가정한다 (fleet_fsm 은 순찰을 명령하지 않는다).
   - robots 파라미터의 이름과 patrol_fire.py 실행 시 네임스페이스(__ns)가 반드시
     일치해야 한다. 그 네임스페이스로 dock_status/battery_state/amcl_pose/
     navigate_to_pose 액션이 발행되어야 이 노드가 로봇을 인식한다.
   - patrol_fire.py 가 f'/{robot_id}/emergency_goal' (std_msgs/String, JSON
     {"x","y","reason"}) 토픽을 구독하도록 연동되어 있어야 실제 출동 명령이
     전달된다 (현재 patrol_fire.py 에는 아직 미구현 - 통합 시 추가 필요).
   - map_pgm / map_yaml 파라미터를 실제 지도 파일로 맞춘다 (nav2 map_server 에
     쓰는 것과 동일한 .pgm + .yaml 쌍. 미지정 시 패키지 share/maps 기본 지도).
     없으면 회색 빈 캔버스로 대체되어 계속 동작한다.
   - irobot_create_msgs 가 설치되어 있어야 한다 (TurtleBot4/Create3 표준 패키지,
     `/robotN/dock_status` 구독에 사용).

2) 실행
     colcon build --packages-select fp_amr_fsm --symlink-install
     source install/setup.bash
     ros2 launch fp_amr_fsm fleet.launch.py          # fleet_fsm + safety_alert_bridge
     ros2 launch fp_amr_fsm fleet.launch.py params_file:=/path/to/my.yaml
     ros2 run fp_amr_fsm fleet_fsm                   # 단독 실행 (기본 파라미터)
   DISPLAY 가 없는 환경(SSH 등)이면 지도 창은 자동으로 비활성화되고 터미널
   로그만 출력된다 (5초 주기 상태 테이블 + 이벤트 즉시 로그).

3) 실습/시연용 이벤트 주입 (실제 웹캠 감지 노드가 아직 없을 때)
     # 위급상황(웹캠이 사람 위치를 map 좌표로 변환해 보냄) 발생시키기
     ros2 topic pub -1 /alert/emergency std_msgs/String \\
       '{data: "{\\"x\\": 1.2, \\"y\\": -0.5}"}'
     # 특정 로봇을 지정해서 출동시키고 싶으면 robot_id 를 함께 보낸다
     ros2 topic pub -1 /alert/emergency std_msgs/String \\
       '{data: "{\\"x\\": 1.2, \\"y\\": -0.5, \\"robot_id\\": \\"robot6\\"}"}'
     # 안전모 미착용 확인 이동 (평시 유지, IDLE 로봇만 배정)
     ros2 topic pub -1 /alert/helmet std_msgs/String \\
       '{data: "{\\"x\\": -1.0, \\"y\\": 2.0}"}'
   실제 연동 시에는 PC3 의 helmet_detector.py/fall_detector.py +
   homography_localizer.py 가 픽셀 좌표를 map 좌표로 변환해 이 두 토픽에
   발행하도록 연결하면 된다.

4) 모니터링 방법 (두 채널, 동시 동작)
   - 터미널: 로그에 5초마다 상태 테이블(로봇 상태 한글 표기 · 배터리 · 위치)이
     찍히고, 미션 배정/도착/점검/충전 등 이벤트는 발생 즉시 로그로 남는다.
   - 지도 창(OpenCV, "Fleet Monitor (map)"): map.pgm 위에 로봇을 상태별 색상
     원으로 표시하고, 상단 배너로 NORMAL(평시)/EMERGENCY(위급상황)를 크게
     표시하며, 출동/안전모 지점과 로봇-목표 연결선, 우측에 로봇별 상세 패널과
     색상 범례를 그린다. 창에서 'q' 키를 누르면 창만 닫히고 노드는 계속 동작.

5) 조정 포인트 (ROS 파라미터, config/fleet_params.yaml)
   - robots: 관제 대상 로봇 네임스페이스 목록
   - map_pgm / map_yaml / viz_scale: 지도 파일 경로와 확대 배율
   - battery_low / battery_resume: 표시/선정 판단용 배터리 임계값(0.0~1.0)
     (patrol_fire.py 자체 임계값과는 독립적 - 표시/로봇 선정 배제용으로만 사용)
   전체 목록과 각 값의 의미는 config/fleet_params.yaml 주석 참고.

── 로봇별 상태머신 (robot2, robot6) ─────────────────────────────
    OFFLINE ─(raw 토픽 수신)→ IDLE
    IDLE ─(출동 배정)→ DISPATCHING(EMERGENCY|HELMET) ─(도착 추론)→ IDLE
    배터리 < LOW ─→ (표시만) RETURNING_LOW_BATTERY → (dock_status.is_docked) → CHARGING
    CHARGING ─(배터리 ≥ RESUME)→ IDLE
    하트비트(raw 토픽) 끊김 ─→ OFFLINE
    순찰(PATROLLING) 은 patrol_fire.py 가 자체적으로 수행하므로 이 FSM 은
    순찰 시작/종료를 명령하거나 표시하지 않는다. IDLE 상태에서 로봇이
    실제로는 순찰 중일 수 있으며, 이는 raw 구동 상태(NAVIGATING)로만 보인다.
    출동 완료는 patrol_fire.py 가 보고하지 않으므로, DISPATCHING 진입 후
    navigating 이 True 를 거쳐 False 로 돌아오면 도착한 것으로 간주하고
    IDLE 로 되돌린다 (nav_status_callback 의 edge 감지).

── 상황(situation) 상태머신 ─────────────────────────────────────
    NORMAL(평시): 로봇들은 patrol_fire.py 로 자체 순찰 중 + 안전모 미착용
        지점 확인 이동(/alert/helmet)만 fleet_fsm 이 배정
    EMERGENCY(위급): 웹캠이 감지한 위치(/alert/emergency {x,y[,robot_id]})로
        지정/최적 로봇에게 즉시 이동 명령. 모든 위급 출동이 끝나면 NORMAL 복귀

── 관제 표시 ────────────────────────────────────────────────────
    1) 터미널: 주기적 상태 테이블 (로봇 상태 한국어 표기, 상황, 배터리, 위치)
    2) map.pgm 창(OpenCV): 지도 위 로봇 위치/상태 색상, 상황 배너,
       위급/안전모 지점 마커, 우측 상태 패널
       (한글은 OpenCV 폰트가 지원하지 않아 화면은 영문, 터미널은 한국어)

발행: /<robot>/emergency_goal (로봇별, JSON {x, y, reason})
구독: /alert/emergency, /alert/helmet,
      /<robot>/battery_state, /<robot>/dock_status, /<robot>/amcl_pose,
      /<robot>/navigate_to_pose/_action/status
==========================================================================
"""

import json
import math
import signal
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import DockStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String

from .map_view import MapView
from .robot_context import COMMAND_QOS, KR_STATE, RobotContext


class FleetFSMNode(Node):

    def __init__(self):
        super().__init__('fleet_fsm_node')

        # === 파라미터 (기본값/의미는 config/fleet_params.yaml 참고) ===
        self.declare_parameter('robots', ['robot2', 'robot6'])
        self.declare_parameter('map_pgm', '')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('viz_scale', 6)
        self.declare_parameter('panel_w', 400)
        self.declare_parameter('battery_low', 0.30)
        self.declare_parameter('battery_resume', 0.90)
        self.declare_parameter('battery_dispatch_min', 0.40)
        self.declare_parameter('offline_timeout', 5.0)
        self.declare_parameter('max_dispatch_attempts', 3)
        self.declare_parameter('tick_period', 1.0)
        self.declare_parameter('table_log_period', 5.0)
        self.declare_parameter('patrol_period', 300.0)

        def p(name):
            return self.get_parameter(name).value
        self.ROBOTS = list(p('robots'))
        self.BATTERY_LOW = float(p('battery_low'))
        self.BATTERY_RESUME = float(p('battery_resume'))
        self.BATTERY_DISPATCH_MIN = float(p('battery_dispatch_min'))
        self.OFFLINE_TIMEOUT = float(p('offline_timeout'))
        self.MAX_DISPATCH_ATTEMPTS = int(p('max_dispatch_attempts'))
        self.TICK_PERIOD = float(p('tick_period'))
        self.TABLE_LOG_PERIOD = float(p('table_log_period'))
        self.PATROL_PERIOD = float(p('patrol_period'))

        # 지도 경로 미지정 시 패키지 share/maps 의 기본 지도 사용
        map_pgm, map_yaml = str(p('map_pgm')), str(p('map_yaml'))
        if not map_pgm:
            map_dir = get_package_share_directory('fp_amr_fsm') + '/maps'
            map_pgm = map_dir + '/final_project.pgm'
            map_yaml = map_dir + '/final_project.yaml'

        self.lock = threading.Lock()
        self.robots = {rid: RobotContext(rid) for rid in self.ROBOTS}
        self.emergency_queue = []   # [{'x','y','robot_id'?}] 미배정 위급 이벤트
        self.helmet_queue = []      # [{'x','y'}] 미배정 안전모 확인 지점
        self.patrol_queue = []      # [{'robot_id'?}] 미배정 순찰 시작 명령
        self.situation = 'NORMAL'   # NORMAL(평시) | EMERGENCY(위급상황)
        self._last_table_log = 0.0
        # 노드 기동 시각 기준으로 첫 자동 순찰은 PATROL_PERIOD 후 배정
        # (기동 직후에는 로봇 discovery/상태 수신이 아직 안 끝나 있을 수 있음)
        self._last_auto_patrol = time.monotonic()

        self.view = MapView(map_pgm, map_yaml, int(p('viz_scale')),
                            int(p('panel_w')), self.get_logger())

        # 로봇별 전용 출동 좌표 토픽. patrol_fire.py 가 이 토픽을 구독해
        # 새 goal 을 받으면 순찰을 중단하고 즉시 이동하도록 연동하는 것이
        # 통합 단계의 몫이며, 이 노드는 발행까지만 책임진다.
        self.emergency_goal_pubs = {
            rid: self.create_publisher(String, f'/{rid}/emergency_goal', COMMAND_QOS)
            for rid in self.ROBOTS
        }
        # 안전모 배달은 emergency_goal 이 아니라 로봇 노드가 구독하는
        # 전용 helmet_goal 토픽으로 보내야 idle 상태에서만 처리된다.
        self.helmet_goal_pubs = {
            rid: self.create_publisher(String, f'/{rid}/helmet_goal', COMMAND_QOS)
            for rid in self.ROBOTS
        }
        # 순찰 시작 명령 (robot_id 미지정 시 배터리 최고 idle 로봇 선정)
        self.patrol_cmd_pubs = {
            rid: self.create_publisher(String, f'/{rid}/patrol_cmd', COMMAND_QOS)
            for rid in self.ROBOTS
        }
        # 응급 조치 완료 신호. 로봇은 이 신호를 받아야 현장 대기를 끝낸다.
        self.emergency_clear_pubs = {
            rid: self.create_publisher(Bool, f'/{rid}/emergency_clear', COMMAND_QOS)
            for rid in self.ROBOTS
        }
        # 안전모 상황 해제 신호 (착용 확인). emergency_clear 와 동일 계약.
        # 로봇 노드가 현장 대기(HELMET_DELIVERY_HOLD)를 조기 종료하는 데
        # 쓸 수 있다 (수신측 미구현이어도 발행은 무해).
        self.helmet_clear_pubs = {
            rid: self.create_publisher(Bool, f'/{rid}/helmet_clear', COMMAND_QOS)
            for rid in self.ROBOTS
        }
        self.status_pub = self.create_publisher(String, '/fleet/status', 10)

        self.create_subscription(String, '/alert/emergency', self.emergency_callback, 10)
        self.create_subscription(String, '/alert/helmet', self.helmet_callback, 10)
        # 디버깅/시연용 명령 주입 토픽:
        #   순찰 시작 (robot_id 생략 시 배터리 최고 idle 로봇):
        #     ros2 topic pub -1 /alert/patrol std_msgs/String '{data: "{}"}'
        #     ros2 topic pub -1 /alert/patrol std_msgs/String \
        #       '{data: "{\"robot_id\": \"robot6\"}"}'
        #   응급 조치 완료 (robot_id 생략 시 위급 출동 중인 모든 로봇에게):
        #     ros2 topic pub -1 /alert/emergency_clear std_msgs/String '{data: "{}"}'
        self.create_subscription(String, '/alert/patrol', self.patrol_callback, 10)
        self.create_subscription(
            String, '/alert/emergency_clear', self.emergency_clear_callback, 10)
        #   안전모 착용 확인(상황 해제): ros2 topic pub -1 /alert/helmet_clear \
        #     std_msgs/String '{data: "{}"}'  (safety_alert_bridge 가
        #     /safety/helmet_state 의 HELMET_CLEAR 를 이 토픽으로 중계한다)
        self.create_subscription(
            String, '/alert/helmet_clear', self.helmet_clear_callback, 10)
        for rid in self.ROBOTS:
            # 로봇의 '구동 상태'는 로봇의 자체 보고(String)를 거치지 않고
            # 각 로봇이 실제로 발행하는 표준 ROS 토픽에서 직접 구독해 판단한다.
            self.create_subscription(
                BatteryState, f'/{rid}/battery_state',
                lambda m, r=rid: self.battery_callback(r, m), qos_profile_sensor_data)
            self.create_subscription(
                DockStatus, f'/{rid}/dock_status',
                lambda m, r=rid: self.dock_status_callback(r, m), qos_profile_sensor_data)
            self.create_subscription(
                PoseWithCovarianceStamped, f'/{rid}/amcl_pose',
                lambda m, r=rid: self.amcl_callback(r, m), 10)
            self.create_subscription(
                GoalStatusArray, f'/{rid}/navigate_to_pose/_action/status',
                lambda m, r=rid: self.nav_status_callback(r, m),
                qos_profile_action_status_default)
            # 로봇 구동 노드가 latched(TRANSIENT_LOCAL)로 발행하는 자체 상태
            self.create_subscription(
                String, f'/{rid}/amr_status',
                lambda m, r=rid: self.amr_status_callback(r, m), COMMAND_QOS)

        self.create_timer(self.TICK_PERIOD, self.tick)

        # 지도 표시 스레드 (DISPLAY 없으면 자동으로 터미널 전용으로 전환)
        self.view.start(self.viz_snapshot)

        self.get_logger().info(
            f'Fleet FSM started. Robots: {self.ROBOTS}, situation=NORMAL(평시)')

    # ------------------------------------------------------------------
    # 수신 콜백
    # ------------------------------------------------------------------
    def mark_alive(self, robot, source):
        """ raw 토픽 수신 = 로봇 스택이 살아있다는 증거 (커스텀 하트비트 불필요).
        모든 raw 콜백에서 호출. OFFLINE -> IDLE 전이도 여기서 처리.
        source 별 수신 시각은 tick 의 스택 세분 진단(어느 노드가 안 떠
        있는지 구분)에 쓴다. """
        robot.last_seen = time.monotonic()
        robot.seen[source] = robot.last_seen
        robot.update_reported_state()
        if robot.fsm_state == 'OFFLINE':
            robot.fsm_state = 'IDLE'
            self.get_logger().info(
                f'[{robot.robot_id}] ONLINE (구동 상태={robot.reported_state})')

    def battery_callback(self, rid, msg):
        with self.lock:
            robot = self.robots.get(rid)
            if robot is None:
                return
            if msg.percentage is not None:
                robot.battery = float(msg.percentage)
            self.mark_alive(robot, 'battery')

    def dock_status_callback(self, rid, msg):
        """ 도킹 여부는 로봇 자체 보고가 아니라 실제 센서 토픽(is_docked) 기준
        (충전 상태 전이 및 '구동 상태' 표시의 근거) """
        with self.lock:
            robot = self.robots.get(rid)
            if robot is None:
                return
            robot.docked = bool(msg.is_docked)
            self.mark_alive(robot, 'dock')

    def nav_status_callback(self, rid, msg):
        """ Nav2 NavigateToPose 액션 상태로 '이동 중' 여부 판단.
        어떤 노드가 goal 을 보냈는지와 무관하게 액션 서버가 직접 발행하는
        표준 토픽이므로, patrol_fire.py 의 자체 보고 없이도 신뢰할 수 있다.
        patrol_fire.py 는 DISPATCH 완료를 이벤트로 알려주지 않으므로, 여기서
        navigating True->False 전이를 직접 감지해 출동 종료를 추론한다. """
        with self.lock:
            robot = self.robots.get(rid)
            if robot is None:
                return
            was_navigating = robot.navigating
            latest = None
            if msg.status_list:
                latest = msg.status_list[-1].status
                robot.navigating = latest in (
                    GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            self.mark_alive(robot, 'nav')

            if robot.fsm_state == 'DISPATCHING':
                if robot.navigating:
                    robot.dispatch_seen_moving = True
                elif was_navigating and robot.dispatch_seen_moving:
                    # True -> False 전이 = 출동 goal 종료. 종료 status 로
                    # 성공(도착)과 실패(접근 불가 좌표, 장애물로 인한 abort,
                    # 취소)를 구분한다.
                    goal, reason = robot.dispatch_goal, robot.dispatch_reason
                    item = robot.dispatch_item
                    robot.fsm_state = 'IDLE'
                    robot.dispatch_goal = None
                    robot.dispatch_reason = None
                    robot.dispatch_item = None
                    robot.dispatch_seen_moving = False
                    if latest == GoalStatus.STATUS_SUCCEEDED:
                        self.get_logger().info(
                            f'[{rid}] 출동 완료({reason}): {goal}. 대기 상태로 복귀.')
                    else:
                        self._handle_dispatch_failure(rid, reason, goal, item, latest)

    def _handle_dispatch_failure(self, rid, reason, goal, item, status):
        """ 출동 goal 이 SUCCEEDED 외의 종료 status(ABORTED=접근 불가/장애물,
        CANCELED 등)로 끝난 경우. 이벤트를 버리지 않고 큐 맨 앞에 되돌려
        다음 tick 에 재배정한다 (같은 좌표가 계속 실패하면 무한 루프가 되므로
        MAX_DISPATCH_ATTEMPTS 회 초과 시 포기하고 에러 로그만 남긴다).
        주의: nav_status_callback 의 lock 안에서 호출되므로 lock 을 잡지 않는다. """
        item = dict(item) if item else {'x': goal[0], 'y': goal[1]}
        item['attempts'] = item.get('attempts', 1) + 1
        # ponytail: 같은 로봇이 재선정될 수 있음(장애물이 치워지면 성공).
        # 좌표 자체가 접근 불가면 attempts 상한이 루프를 끊는다.
        if item['attempts'] <= self.MAX_DISPATCH_ATTEMPTS:
            queue = (self.emergency_queue if reason == 'EMERGENCY'
                     else self.helmet_queue)
            queue.insert(0, item)
            self.get_logger().error(
                f'[{rid}] 출동 실패({reason}, status={status}): {goal}. '
                f'재배정 예정 (시도 {item["attempts"]}/{self.MAX_DISPATCH_ATTEMPTS})')
        else:
            self.get_logger().error(
                f'[{rid}] 출동 실패({reason}, status={status}): {goal}. '
                f'{self.MAX_DISPATCH_ATTEMPTS}회 시도 후 포기 - 좌표 접근 불가로 판단.')

    def amcl_callback(self, rid, msg):
        p = msg.pose.pose.position
        with self.lock:
            robot = self.robots.get(rid)
            if robot is None:
                return
            robot.pose = [round(p.x, 3), round(p.y, 3)]
            self.mark_alive(robot, 'amcl')

    def emergency_callback(self, msg):
        """ 위급상황: 웹캠 감지 위치 {x, y[, robot_id]} -> 상황 EMERGENCY 전환 + 큐 """
        try:
            data = json.loads(msg.data)
            item = {'x': float(data['x']), 'y': float(data['y'])}
            if data.get('robot_id'):
                item['robot_id'] = data['robot_id']  # 특정 로봇 지정 출동
        except Exception as e:
            self.get_logger().warn(f'Invalid emergency message: {e}')
            return
        with self.lock:
            self.emergency_queue.append(item)
        self.set_situation('EMERGENCY',
                           f"웹캠 감지 위치 ({item['x']:.2f}, {item['y']:.2f})")

    def helmet_callback(self, msg):
        """ 평시 이벤트: 안전모 미착용 감지 위치 {x, y} -> 확인 이동 큐 (상황 유지) """
        try:
            data = json.loads(msg.data)
            item = {'x': float(data['x']), 'y': float(data['y'])}
        except Exception as e:
            self.get_logger().warn(f'Invalid helmet message: {e}')
            return
        with self.lock:
            self.helmet_queue.append(item)
        self.get_logger().warn(
            f"안전모 미착용 감지: ({item['x']:.2f}, {item['y']:.2f}) - 확인 이동 배정 예정")

    def amr_status_callback(self, rid, msg):
        handover = False
        with self.lock:
            robot = self.robots.get(rid)
            if robot is None:
                return
            prev = robot.amr_status
            robot.amr_status = msg.data
            self.mark_alive(robot, 'amr')
            # ── 순찰 교대 감지 ──
            # PATROL → RETURNING 전이는 로봇 노드가 순찰 중 배터리 부족
            # (BATTERY_LOW_THRESHOLD 미만)으로 순찰을 중단하고 자체 복귀를
            # 시작했다는 뜻 (위급 선점이면 PATROL → EMERGENCY 로 가므로
            # 이 전이와 구분된다). 복귀하는 로봇 대신 다른 가용 로봇에게
            # 순찰을 이어받게 큐에 넣는다. 대체 로봇 선정은 assign_patrol/
            # select_patrol_robot 이 담당하며, 복귀 중인 이 로봇은
            # amr_status=RETURNING + 배터리 < BATTERY_DISPATCH_MIN 이라
            # 후보에서 자동 제외된다. 가용 로봇이 없으면 큐에 남아 있다가
            # 생기는 즉시 배정된다.
            if prev == 'PATROL' and msg.data == 'RETURNING':
                if self.situation == 'NORMAL' and not self.patrol_queue:
                    self.patrol_queue.append({})
                    handover = True
        if handover:
            batt = (f'{robot.battery*100:.0f}%'
                    if robot.battery is not None else '?')
            self.get_logger().warn(
                f'[{rid}] 순찰 중 배터리 부족 복귀 감지(배터리 {batt}) - '
                f'다른 가용 로봇에게 순찰 교대 배정 예정')

    def patrol_callback(self, msg):
        """ 순찰 시작 명령 주입: {} 또는 {robot_id}.
        robot_id 미지정이면 배터리가 가장 높은 idle 로봇을 선정해 보낸다. """
        item = {}
        if msg.data.strip():
            try:
                data = json.loads(msg.data)
                if data.get('robot_id'):
                    item['robot_id'] = data['robot_id']
            except Exception as e:
                self.get_logger().warn(f'Invalid patrol message: {e}')
                return
        with self.lock:
            self.patrol_queue.append(item)
        self.get_logger().info(
            f"순찰 시작 요청 수신 (robot_id={item.get('robot_id', '자동(배터리 최고)')})")

    def emergency_clear_callback(self, msg):
        """ 응급 조치 완료: {} 또는 {robot_id}. robot_id 미지정이면 위급 출동
        중(DISPATCHING/EMERGENCY)인 모든 로봇에게 clear 를 내려보낸다. """
        target_id = None
        if msg.data.strip():
            try:
                data = json.loads(msg.data)
                target_id = data.get('robot_id')
            except Exception as e:
                self.get_logger().warn(f'Invalid emergency_clear message: {e}')
                return
        with self.lock:
            if target_id:
                targets = [target_id] if target_id in self.robots else []
            else:
                targets = [r.robot_id for r in self.robots.values()
                           if r.fsm_state == 'DISPATCHING'
                           and r.dispatch_reason == 'EMERGENCY']
            # 로봇이 이미 도착해 fleet 쪽에서는 IDLE 로 돌아갔지만(도착 추론)
            # 로봇은 현장에서 clear 를 기다리는 경우가 있으므로, 출동 중인
            # 로봇이 하나도 없으면 전체 로봇에게 보낸다 (수신측이 무해하게 처리).
            if not targets:
                targets = list(self.robots.keys())
        clear = Bool()
        clear.data = True
        for rid in targets:
            self.emergency_clear_pubs[rid].publish(clear)
            self.get_logger().info(f'[{rid}] 응급 조치 완료(emergency_clear) 발행')

    def helmet_clear_callback(self, msg):
        """ 안전모 착용 확인(상황 해제): {} 또는 {robot_id}.
        emergency_clear 와 동일 패턴 + 아직 배정 전인 helmet 큐도 함께
        폐기한다 (착용이 확인된 지점으로 뒤늦게 확인 이동할 이유가 없음). """
        target_id = None
        if msg.data.strip():
            try:
                data = json.loads(msg.data)
                target_id = data.get('robot_id')
            except Exception as e:
                self.get_logger().warn(f'Invalid helmet_clear message: {e}')
                return
        with self.lock:
            dropped = len(self.helmet_queue)
            self.helmet_queue.clear()
            if target_id:
                targets = [target_id] if target_id in self.robots else []
            else:
                targets = [r.robot_id for r in self.robots.values()
                           if r.fsm_state == 'DISPATCHING'
                           and r.dispatch_reason == 'HELMET']
            # 로봇이 이미 도착해 fleet 쪽에서는 IDLE 로 돌아갔지만 로봇은
            # 현장 대기 중일 수 있으므로, 대상이 없으면 전체에게 보낸다.
            if not targets:
                targets = list(self.robots.keys())
        if dropped:
            self.get_logger().info(
                f'안전모 착용 확인 - 미배정 helmet 이벤트 {dropped}건 폐기')
        clear = Bool()
        clear.data = True
        for rid in targets:
            self.helmet_clear_pubs[rid].publish(clear)
            self.get_logger().info(f'[{rid}] 안전모 해제(helmet_clear) 발행')

    def set_situation(self, situation, reason=''):
        with self.lock:
            if self.situation == situation:
                return
            self.situation = situation
        kr = '위급상황' if situation == 'EMERGENCY' else '평시'
        msg = f'●●● 상황 전환: {situation}({kr}) {reason}'

        # 아예 error() 하나로 통일 (위급상황의 심각도가 높으니 ERROR로 항상 출력하는 것도 가능)
        if situation == 'EMERGENCY':
            self.get_logger().error(msg)
        else:
            self.get_logger().info(msg)

    # ------------------------------------------------------------------
    # FSM 판단 주기
    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()  # 일정하게 흐름. 신뢰 가능 <-> 리얼타임
        with self.lock:
            # 1) 하트비트 기반 OFFLINE 판정
            for robot in self.robots.values():
                if (robot.fsm_state != 'OFFLINE' and robot.last_seen is not None
                        and now - robot.last_seen > self.OFFLINE_TIMEOUT):
                    robot.fsm_state = 'OFFLINE'
                    self.get_logger().error(
                        f'[{robot.robot_id}] 하트비트 끊김 -> OFFLINE')

            # 2) 순찰 시작/종료는 patrol_fire.py 가 자체 처리하므로 이 FSM 은
            # 관여하지 않는다. 여기서는 하트비트 단절(OFFLINE)만 감지한다.

        # 2.5) 로봇 스택 세분 진단: 어떤 토픽/노드가 살아있는지 조합해
        # "무엇이 안 떠 있는지"를 구분한다 (출동 선정에서도 제외).
        #   battery O + amcl X                       -> NO_LOCALIZATION
        #   battery O + amcl O + planner/smoother X  -> NO_NAV2
        self.diagnose_stacks(now)

        # 3) 배터리 상태 갱신 (표시/선정 배제용. 복귀 명령은 내리지 않음 -
        # patrol_fire.py 가 스스로 배터리 부족을 감지해 복귀·도킹한다)
        self.update_battery_state()

        # 4) 위급 출동 배정 (최우선, 안전모 확인 이동 중인 로봇 선점 허용)
        self.assign_dispatch(self.emergency_queue, 'EMERGENCY', preempt_helmet=True)

        # 5) 평시: 안전모 확인 이동 배정 (IDLE 로봇만)
        self.assign_dispatch(self.helmet_queue, 'HELMET', preempt_helmet=False)

        # 5.3) 자동 순찰 주입: PATROL_PERIOD 마다 순찰 큐에 자동으로 이벤트를
        # 넣는다 (기존에는 /alert/patrol 수동 주입이 있어야만 순찰 시작).
        self.auto_patrol(now)

        # 5.5) 순찰 시작 배정 (robot_id 미지정 시 배터리 최고 idle 로봇)
        self.assign_patrol()

        # 6) 상황 복귀 판정: 위급 큐 비었고 위급 출동 중 로봇 없음 -> NORMAL
        with self.lock:
            emergency_active = bool(self.emergency_queue) or any(
                r.fsm_state == 'DISPATCHING' and r.dispatch_reason == 'EMERGENCY'
                for r in self.robots.values())
        if not emergency_active:
            self.set_situation('NORMAL', '(위급 대응 완료)')

        # 7) 터미널 상태 테이블 (주기 출력)
        if now - self._last_table_log >= self.TABLE_LOG_PERIOD:
            self._last_table_log = now
            self.log_status_table()
        self.publish_status()

    def diagnose_stacks(self, now):
        """ 로봇별 스택 세분 진단.
        - battery_state 는 Create3 베이스가 항상 주기 발행하므로 '베이스 생존'
          신호로 쓴다 (OFFLINE_TIMEOUT 내 수신 = 신선).
        - amcl_pose 는 로봇이 움직여 필터가 갱신될 때만 발행되므로 신선도가
          아니라 '한 번이라도 수신했는가'로 판단한다 (정지 로봇 오진 방지).
        - planner_server/smoother_server 는 토픽이 없으므로 ROS 노드 그래프
          (get_node_names_and_namespaces)에서 해당 네임스페이스 노드 존재로
          확인한다. """
        running = set(self.get_node_names_and_namespaces())
        with self.lock:
            for robot in self.robots.values():
                fault = None
                if robot.fsm_state != 'OFFLINE':
                    batt_fresh = (robot.seen.get('battery') is not None
                                  and now - robot.seen['battery'] <= self.OFFLINE_TIMEOUT)
                    amcl_seen = 'amcl' in robot.seen
                    ns = f'/{robot.robot_id}'
                    if batt_fresh and not amcl_seen:
                        fault = 'NO_LOCALIZATION'
                    elif batt_fresh and amcl_seen and not (
                            ('planner_server', ns) in running
                            and ('smoother_server', ns) in running):
                        fault = 'NO_NAV2'
                if fault != robot.stack_fault:
                    if fault:
                        detail = ('localization(amcl) 노드 미실행 추정: '
                                  'battery_state 는 수신되나 amcl_pose 미수신'
                                  if fault == 'NO_LOCALIZATION' else
                                  'nav2 미실행 추정: planner_server/smoother_server '
                                  '노드가 그래프에 없음')
                        self.get_logger().error(f'[{robot.robot_id}] {fault} - {detail}')
                    else:
                        self.get_logger().info(
                            f'[{robot.robot_id}] 스택 진단 정상 복귀')
                    robot.stack_fault = fault

    def update_battery_state(self):
        """ 배터리 임계값에 따라 표시 상태만 갱신한다. patrol_fire.py 가
        복귀/도킹을 자체 수행하므로 이 노드는 명령을 내리지 않고, 그 결과를
        raw 토픽(dock_status/battery_state)으로 관찰해 표시에만 반영한다. """
        for robot in self.robots.values():
            with self.lock:
                b = robot.battery
                state = robot.fsm_state
                docked = robot.docked
                charging = robot.charging
            if b is None or state == 'OFFLINE':
                continue

            with self.lock:
                if (not charging and b < self.BATTERY_LOW
                        and state not in ('RETURNING_LOW_BATTERY', 'CHARGING', 'DISPATCHING')):
                    robot.fsm_state = 'RETURNING_LOW_BATTERY'
                    robot.charging = True
                    self.get_logger().warn(
                        f'[{robot.robot_id}] 배터리 {b*100:.0f}% < '
                        f'{self.BATTERY_LOW*100:.0f}% - 자체 복귀 예상 (표시만 갱신)')
                    continue

                # 도킹 완료 -> 충전 중
                if robot.charging and robot.fsm_state == 'RETURNING_LOW_BATTERY' \
                        and docked:
                    robot.fsm_state = 'CHARGING'
                # 충전 완료 -> 대기(순찰은 로봇이 자체 재개)
                if robot.charging and robot.fsm_state == 'CHARGING' \
                        and b >= self.BATTERY_RESUME:
                    robot.charging = False
                    robot.fsm_state = 'IDLE'
                    self.get_logger().info(
                        f'[{robot.robot_id}] 충전 완료({b*100:.0f}%). 대기 상태로 전환.')

    def publish_status(self):
        """ render_frame() 이 OpenCV 로 그리던 상태를 JSON 토픽으로도 발행.
        lock 안에서는 dict 구성(복사)만 하고, publish 는 lock 밖에서 수행 """
        now = time.monotonic()
        with self.lock:
            payload = {
                'situation': self.situation,
                'emergency_queue': list(self.emergency_queue),
                'helmet_queue': list(self.helmet_queue),
                'patrol_queue': list(self.patrol_queue),
                'robots': {
                    r.robot_id: {
                        'fsm_state': r.fsm_state,
                        'display_state': r.display_state(),
                        'reported_state': r.reported_state,
                        'stack_fault': r.stack_fault,
                        'amr_status': r.amr_status,
                        'docked': r.docked,
                        'navigating': r.navigating,
                        'dispatch_seen_moving': r.dispatch_seen_moving,
                        'battery': r.battery,
                        'pose': r.pose,
                        'dispatch_goal': r.dispatch_goal,
                        'dispatch_reason': r.dispatch_reason,
                        'last_seen_age': (now - r.last_seen) if r.last_seen else None,
                    } for r in self.robots.values()
                },
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def assign_dispatch(self, queue, reason, preempt_helmet):
        with self.lock:
            item = queue.pop(0) if queue else None
        if item is None:
            return
        robot = self.select_dispatch_robot(item, preempt_helmet)
        if robot is None:
            with self.lock:
                queue.insert(0, item)  # 가용 로봇 생길 때까지 재시도
            return
        self.publish_goal(robot, item['x'], item['y'], reason)
        with self.lock:
            robot.fsm_state = 'DISPATCHING'
            robot.dispatch_goal = [item['x'], item['y']]
            robot.dispatch_reason = reason
            robot.dispatch_item = item  # 실패 시 재큐잉용 원본 보관
            robot.dispatch_seen_moving = False

    def select_dispatch_robot(self, item, preempt_helmet):
        """ 출동 로봇 선정.
        - 이벤트에 robot_id 가 지정되면 해당 로봇 (가용할 때만)
        - 아니면 IDLE 우선. 위급(EMERGENCY)이면 안전모 확인 이동 중인
          로봇(DISPATCHING/HELMET) 선점도 허용. 최단 거리 순
        순찰 자체는 patrol_fire.py 가 자체 수행해 이 FSM 이 모르는 상태이므로
        (fsm_state 는 항상 IDLE 로 보임) 순찰 선점이라는 개념 자체가 없다 -
        출동 명령은 로봇의 현재 위치와 무관하게 그냥 내려보내면 patrol_fire.py
        가 (연동 완료 시) 자체적으로 순찰을 중단하고 이동한다. """
        gx, gy = item['x'], item['y']

        def available(r, states):
            # stack_fault(측위/nav2 미실행) 로봇은 goal 을 받아도 이동할 수
            # 없으므로 선정에서 제외한다.
            # 배터리가 BATTERY_DISPATCH_MIN 미만인 로봇도 제외 - 로봇 자체
            # 복귀 임계값(30%) 부근에서 배정하면 출동 직후 배터리 복귀로
            # 이탈한다 (배터리 미보고 로봇은 판단 불가이므로 통과).
            return (r.fsm_state in states and not r.charging
                    and r.stack_fault is None
                    and (r.battery is None
                         or r.battery >= self.BATTERY_DISPATCH_MIN))

        def distance(r):
            if r.pose is None:
                return float('inf')
            return math.hypot(r.pose[0] - gx, r.pose[1] - gy)

        with self.lock:
            forced_id = item.get('robot_id')
            if forced_id:
                robot = self.robots.get(forced_id)
                if robot is not None and available(robot, ('IDLE', 'DISPATCHING')):
                    return robot
                self.get_logger().warn(
                    f'지정 로봇 {forced_id} 사용 불가. 자동 선정으로 전환.')

            idle_pool = [r for r in self.robots.values() if available(r, ('IDLE',))]
            if not preempt_helmet:
                # 안전모 확인은 로봇 노드가 idle 상태에서만 실행하므로,
                # amr_status 로 순찰(PATROL) 중인 로봇은 후보에서 제외한다.
                # (위급은 로봇 노드가 순찰을 즉시 선점하므로 제외하지 않음)
                idle_pool = [r for r in idle_pool
                             if r.amr_status in (None, 'IDLE')]
            if idle_pool:
                return min(idle_pool, key=distance)

            if preempt_helmet:
                helmet_pool = [r for r in self.robots.values()
                               if available(r, ('DISPATCHING',))
                               and r.dispatch_reason == 'HELMET']
                if helmet_pool:
                    return min(helmet_pool, key=distance)
        return None

    def publish_goal(self, robot, x, y, reason):
        """ 로봇 전용 토픽에 좌표 발행. 위급은 emergency_goal, 안전모는
        helmet_goal 로 보낸다 (로봇 노드가 우선순위를 다르게 처리:
        emergency 는 즉시 선점, helmet 은 idle 상태에서만 실행). """
        payload = {'x': x, 'y': y, 'reason': reason,
                   'robot_id': robot.robot_id, 'timestamp': time.time()}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        pubs = self.helmet_goal_pubs if reason == 'HELMET' else self.emergency_goal_pubs
        pubs[robot.robot_id].publish(msg)
        self.get_logger().info(
            f'[{robot.robot_id}] 출동 좌표 발행: ({x:.2f}, {y:.2f}) reason={reason}')

    def auto_patrol(self, now):
        """ PATROL_PERIOD 주기 자동 순찰 배정.
        조건이 하나라도 어긋나면 이번 주기는 건너뛴다 (큐에 쌓아두지 않음 -
        수동 주입과 달리 자동 주입은 밀린 것을 나중에 몰아서 처리할 이유가 없다):
          - 평시(NORMAL) 일 것 (위급상황 중 순찰 배정 금지)
          - 순찰 큐가 비어 있을 것 (수동 주입 대기분과 중복 방지)
          - 이미 순찰 중(amr_status=PATROL)인 로봇이 없을 것 """
        if self.PATROL_PERIOD <= 0:
            return
        if now - self._last_auto_patrol < self.PATROL_PERIOD:
            return
        with self.lock:
            if (self.situation != 'NORMAL' or self.patrol_queue
                    or any(r.amr_status == 'PATROL' for r in self.robots.values())):
                # 주기는 소모하되 이벤트는 넣지 않는다. 다음 주기에 재평가.
                self._last_auto_patrol = now
                return
            self.patrol_queue.append({})
            self._last_auto_patrol = now
        self.get_logger().info(
            f'자동 순찰 주기 도래({self.PATROL_PERIOD:.0f}s) - '
            f'배터리 최고 idle 로봇에게 순찰 배정 예정')

    def assign_patrol(self):
        with self.lock:
            item = self.patrol_queue.pop(0) if self.patrol_queue else None
        if item is None:
            return
        robot = self.select_patrol_robot(item.get('robot_id'))
        if robot is None:
            with self.lock:
                self.patrol_queue.insert(0, item)  # 가용 로봇 생길 때까지 재시도
            return
        payload = {'robot_id': robot.robot_id, 'timestamp': time.time()}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.patrol_cmd_pubs[robot.robot_id].publish(msg)
        battery = f'{robot.battery*100:.0f}%' if robot.battery is not None else '?'
        self.get_logger().info(
            f'[{robot.robot_id}] 순찰 시작 명령 발행 (배터리 {battery})')

    def select_patrol_robot(self, forced_id=None):
        """ 순찰 로봇 선정: robot_id 지정 시 해당 로봇(가용할 때만),
        미지정 시 idle 로봇 중 배터리가 가장 높은 로봇. """
        def idle_available(r):
            return (r.fsm_state == 'IDLE' and not r.charging
                    and r.stack_fault is None
                    and r.amr_status in (None, 'IDLE')
                    and (r.battery is None
                         or r.battery >= self.BATTERY_DISPATCH_MIN))

        with self.lock:
            if forced_id:
                robot = self.robots.get(forced_id)
                if robot is not None and idle_available(robot):
                    return robot
                self.get_logger().warn(
                    f'지정 로봇 {forced_id} 순찰 배정 불가. 자동 선정으로 전환.')

            pool = [r for r in self.robots.values() if idle_available(r)]
            if pool:
                # 배터리 미보고 로봇(-1 취급)보다 보고된 로봇을 우선한다.
                return max(pool, key=lambda r: r.battery
                           if r.battery is not None else -1.0)
        return None

    # ------------------------------------------------------------------
    # 터미널 상태 테이블
    # ------------------------------------------------------------------
    def log_status_table(self):
        with self.lock:
            situation = self.situation
            n_emg, n_helmet = len(self.emergency_queue), len(self.helmet_queue)
            rows = []
            for r in self.robots.values():
                batt = f'{r.battery*100:5.1f}%' if r.battery is not None else '  ?  '
                pose = (f'({r.pose[0]:6.2f},{r.pose[1]:6.2f})'
                        if r.pose else '(  ?  ,  ?  )')
                kr = KR_STATE.get(r.display_state(), r.display_state())
                rows.append(f'  {r.robot_id:8s} {kr:14s} 배터리 {batt}  위치 {pose}')
        kr_sit = '위급상황' if situation == 'EMERGENCY' else '평시'
        lines = [f'━━ 관제 상태: {situation}({kr_sit})'
                 f'  위급대기 {n_emg} / 안전모대기 {n_helmet} ━━'] + rows
        self.get_logger().info('\n'.join(lines))

    # ------------------------------------------------------------------
    # 지도 화면 스냅샷 (그리기 자체는 map_view.MapView 가 담당)
    # ------------------------------------------------------------------
    def viz_snapshot(self):
        """ MapView 가 매 프레임 그릴 데이터를 lock 안에서 복사해 반환 """
        with self.lock:
            return {
                'situation': self.situation,
                'emg_pts': [(e['x'], e['y']) for e in self.emergency_queue],
                'helmet_pts': [(h['x'], h['y']) for h in self.helmet_queue],
                'robots': [{
                    'id': r.robot_id, 'pose': r.pose,
                    'state': r.display_state(),
                    'battery': r.battery,
                    'goal': list(r.dispatch_goal) if r.dispatch_goal else None,
                    'reason': r.dispatch_reason,
                } for r in self.robots.values()],
            }

    def destroy_node(self):
        self.view.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = FleetFSMNode()
    # ros2 launch 는 SIGINT 로 종료시킨다. humble rclpy 는 컨텍스트를 먼저
    # 닫고(spin 이 ExternalShutdownException 을 던짐) 그 뒤 KeyboardInterrupt
    # 를 비동기로 한 번 더 올리므로, 정리 단계에서도 잡아야 traceback 없이 죽는다.
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 정리 중 추가 SIGINT 가 들어오면 GUI 스레드 join 이 끊겨 Qt 가
        # abort(-6) 하므로, 정리 시작과 동시에 SIGINT 를 무시한다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
