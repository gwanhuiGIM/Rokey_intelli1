#!/usr/bin/env python3
"""
AMR 스마트 공장 순찰 FSM(상태머신) 노드.

지침서 '시나리오별 노드 운영 지침서'의 로봇(PC1/PC2) 측 흐름을 하나의
상태머신으로 구현한다 (시나리오 2~7, 상태값 0.2절 기준).

    IDLE
     └→ PATROLLING ──(설비 도착)──→ INSPECTING
             ↑                          │ ArUco 인식 → ticket_info 발행
             │                          │ 판정 수신: CHECKED → 순찰 계속
             │                          │           NEED_REPLACEMENT → 교체 큐 등록
             └──────(다음 설비)─────────┘
     한 바퀴 완료 & 큐 존재
     └→ RETURNING_TO_BASE → ARRIVED_BASE → PART_CHECK(PART_CHECKED/PART_LOADED)
         → MOVING_TO_FACILITY → REPLACING → REPLACED/FAILED
         → RETURNING_TO_BASE (다음 큐 항목 또는 순찰 재개)
     /alert/emergency 수신 시 어느 상태에서든 EMERGENCY_RESPONSE 로 선점,
     완료 후 중단 지점 성격에 맞는 상태로 복귀 (순찰 중이었으면 PATROLLING,
     교체 흐름 중이었으면 RETURNING_TO_BASE 부터 재시작)

외부 연동 (모두 없어도 단독 시연 가능하도록 폴백 내장):
  - 발행: /ticket_info                (String/JSON, 설비 점검 요청 → PC4 equipment_db_node)
          /facility/replacement_result (String/JSON, 교체 성공/실패)
          /fleet/robot_status          (String/JSON, 상태 전이마다)
  - 구독: /equipment/judgment          (String/JSON {facility_id, need_replacement})
          → JUDGMENT_TIMEOUT 내 미수신 시 LOCAL 폴백 판정 사용
          /alert/emergency             (String/JSON {x, y}) → 긴급 출동 선점

maintenance_actuator 는 지침서 가정대로 '교체 수행 장치가 장착되어 있다'고
보고 REPLACE_DURATION 동안 수행하는 것으로 모사한다.
"""

import json
import threading
import time
from enum import Enum, auto

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class State(Enum):
    IDLE = auto()                # 초기화 (도킹 확인, 초기 포즈, Nav2 대기, 언도킹)
    PATROLLING = auto()          # 다음 설비 waypoint 로 이동 (MOVING_TO_TARGET)
    INSPECTING = auto()          # 설비 앞 ArUco 인식 + ticket_info + 판정 대기
    RETURNING_TO_BASE = auto()   # 베이스 복귀 이동
    ARRIVED_BASE = auto()        # 베이스 도착. 다음 행동(교체/순찰 재개) 분기
    PART_CHECK = auto()          # 재고 확인(PART_CHECKED) 및 적재(PART_LOADED)
    MOVING_TO_FACILITY = auto()  # 교체 대상 설비로 이동
    REPLACING = auto()           # 설비 재확인 + maintenance_actuator 교체 수행
    EMERGENCY_RESPONSE = auto()  # 긴급 출동 (최우선 선점)


class PatrolFSMNode(Node):

    # === 로봇/베이스 설정 ===
    BASE_POSE = ([0.0, 0.0], TurtleBot4Directions.NORTH)   # 베이스(적재/복귀 지점)
    INITIAL_POSE_POSITION = [0.0, 0.0]
    INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

    # === 순찰 설비 목록 ===
    # marker_id: 설비에 부착된 ArUco ID, part_type: 교체 시 필요한 물품 종류
    # 실제 맵 좌표/마커 ID 로 교체해서 사용할 것
    FACILITIES = [
        {'facility_id': 'FAC_01', 'waypoint': [-0.02, -1.39],
         'direction': TurtleBot4Directions.NORTH, 'marker_id': 0, 'part_type': 'FILTER'},
        {'facility_id': 'FAC_02', 'waypoint': [-2.77, -1.29],
         'direction': TurtleBot4Directions.SOUTH, 'marker_id': 1, 'part_type': 'BELT'},
        {'facility_id': 'FAC_03', 'waypoint': [-0.29, -0.22],
         'direction': TurtleBot4Directions.EAST, 'marker_id': 2, 'part_type': 'FILTER'},
    ]

    # === ArUco 설정 ===
    ARUCO_DICT = cv2.aruco.DICT_5X5_50
    DETECT_WINDOW = 5.0        # 설비 앞 마커 탐색 제한 시간(초). 초과 시 FAILED_CHECK
    DETECT_MIN_HITS = 3        # aruco_stabilizer: 연속 N 프레임 감지 시 확정 (오검출 방지)

    # === 판정/교체 설정 ===
    JUDGMENT_TIMEOUT = 3.0     # PC4 판정 대기 시간(초). 초과 시 로컬 폴백 판정
    # PC4(equipment_db_node) 미연결 단독 시연용 폴백: 교체 필요 여부
    LOCAL_FALLBACK_NEED_REPLACEMENT = {'FAC_01': False, 'FAC_02': True, 'FAC_03': False}
    INITIAL_STOCK = {'FILTER': 2, 'BELT': 1}   # inventory_manager 폴백용 로컬 재고
    LOAD_DURATION = 3.0        # 베이스 적재(PART_LOADED) 소요 가정(초)
    REPLACE_DURATION = 5.0     # maintenance_actuator 교체 수행 가정(초)
    EMERGENCY_HOLD = 5.0       # 긴급 지점 도착 후 현장 대기(초)

    def __init__(self):
        super().__init__('patrol_fsm_node')

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        ns = self.get_namespace()
        self.robot_id = ns.strip('/') or 'robot1'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        # --- 공유 상태 (lock 보호) ---
        self.rgb_seq = 0
        self.last_aruco_ids = None
        self.judgments = {}        # facility_id -> need_replacement(bool)
        self.emergency_goal = None  # (x, y) 또는 None

        # --- FSM 내부 상태 (FSM 스레드 전용, lock 불필요) ---
        self.state = State.IDLE
        self.facility_idx = 0          # 이번 바퀴에서 다음에 점검할 설비 인덱스
        self.replace_queue = []        # NEED_REPLACEMENT 설비 dict 목록 (duplicate_guard 포함)
        self.stock = dict(self.INITIAL_STOCK)
        self.resume_state = State.PATROLLING  # 긴급 대응 후 복귀 지점

        dictionary = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.aruco_detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 발행: 지침서 토픽 구조 (fleet/설비 토픽은 전역, 디버그 영상은 네임스페이스)
        self.status_pub = self.create_publisher(String, '/fleet/robot_status', 10)
        self.ticket_pub = self.create_publisher(String, '/ticket_info', 10)
        self.result_pub = self.create_publisher(String, '/facility/replacement_result', 10)
        self.rgb_pub = self.create_publisher(ROSImage, f'{ns}/patrol/rgb_processed', 1)

        # 구독
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, img_qos)
        self.create_subscription(String, '/equipment/judgment', self.judgment_callback, 10)
        self.create_subscription(String, '/alert/emergency', self.emergency_callback, 10)

        self.navigator = TurtleBot4Navigator()

        # FSM 은 순차/블로킹 로직이므로 별도 스레드에서 실행
        self.fsm_stop = threading.Event()
        self.fsm_thread = threading.Thread(target=self.run_fsm, daemon=True)
        self.fsm_thread.start()

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg):
        pass  # FSM 은 마커 ID 만 사용. 위치 추정이 필요해지면 K/D 저장 추가

    def rgb_callback(self, rgb_msg):
        try:
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb is None or rgb.size == 0:
                return

            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

            with self.lock:
                self.rgb_seq += 1
                self.last_aruco_ids = ids

            overlay = rgb.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
            cv2.putText(overlay, f'STATE: {self.state.name}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header.stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(overlay_msg)
        except Exception as e:
            self.get_logger().error(f'RGB callback failed: {e}')

    def judgment_callback(self, msg):
        """ PC4 equipment_db_node 판정 수신: {facility_id, need_replacement} """
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.judgments[data['facility_id']] = bool(data['need_replacement'])
        except Exception as e:
            self.get_logger().warn(f'Invalid judgment message: {e}')

    def emergency_callback(self, msg):
        """ PC3/PC4 긴급 이벤트 수신: {x, y}. FSM 루프가 상태 선점 처리 """
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.emergency_goal = (float(data['x']), float(data['y']))
            self.get_logger().warn(
                f"EMERGENCY received: map=({data['x']:.2f}, {data['y']:.2f})")
        except Exception as e:
            self.get_logger().warn(f'Invalid emergency message: {e}')

    # ------------------------------------------------------------------
    # 공용 헬퍼
    # ------------------------------------------------------------------
    def set_state(self, new_state, detail=''):
        """ 상태 전이 + /fleet/robot_status 발행 (지침서: 전이마다 대시보드 갱신) """
        self.state = new_state
        msg = String()
        msg.data = json.dumps({
            'robot_id': self.robot_id,
            'state': new_state.name,
            'detail': detail,
            'timestamp': time.time(),
        }, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(f'[FSM] -> {new_state.name}' + (f' ({detail})' if detail else ''))

    def emergency_pending(self):
        with self.lock:
            return self.emergency_goal is not None

    def navigate_to(self, position, direction, label):
        """ Nav2 goal 전송 후 완료까지 폴링.
        startToPose 대신 goToPose+폴링을 쓰는 이유: 긴급 이벤트/종료 요청이
        오면 진행 중인 goal 을 즉시 취소(선점)할 수 있어야 하기 때문.
        반환: 'ok' | 'failed' | 'preempted' | 'stopped' """
        pose = self.navigator.getPoseStamped(position, direction)
        self.get_logger().info(f'Navigating to {label}: {position}')
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            if self.fsm_stop.is_set():
                self.navigator.cancelTask()
                return 'stopped'
            if self.emergency_pending() and self.state != State.EMERGENCY_RESPONSE:
                self.navigator.cancelTask()
                return 'preempted'
            time.sleep(0.2)

        result = self.navigator.getResult()
        return 'ok' if result == TaskResult.SUCCEEDED else 'failed'

    def wait_for_marker(self, marker_id, window):
        """ ArUco 안정화 감지: window 초 안에 marker_id 가 연속
        DETECT_MIN_HITS 프레임 보이면 True (지침서 aruco_stabilizer 역할) """
        start = time.monotonic()
        hits = 0
        last_seq = -1
        while (time.monotonic() - start < window
                and not self.fsm_stop.is_set() and not self.emergency_pending()):
            with self.lock:
                seq = self.rgb_seq
                ids = self.last_aruco_ids
            if seq != last_seq:
                last_seq = seq
                id_list = ids.flatten().tolist() if ids is not None else []
                if marker_id in id_list:
                    hits += 1
                    if hits >= self.DETECT_MIN_HITS:
                        return True
                else:
                    hits = 0  # 연속성 끊김 -> 초기화
            time.sleep(0.05)
        return False

    def get_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=0.5))
            return (round(tf.transform.translation.x, 3),
                    round(tf.transform.translation.y, 3))
        except Exception:
            return None

    def publish_ticket(self, facility):
        ticket = {
            'facility_id': facility['facility_id'],
            'marker_id': facility['marker_id'],
            'robot_id': self.robot_id,
            'robot_pose': self.get_robot_xy(),
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(ticket, ensure_ascii=False)
        self.ticket_pub.publish(msg)

    def wait_judgment(self, facility_id):
        """ PC4 판정 대기. 시간 초과 시 로컬 폴백 판정 반환 """
        deadline = time.monotonic() + self.JUDGMENT_TIMEOUT
        while time.monotonic() < deadline and not self.fsm_stop.is_set():
            with self.lock:
                if facility_id in self.judgments:
                    return self.judgments.pop(facility_id)
            time.sleep(0.1)
        fallback = self.LOCAL_FALLBACK_NEED_REPLACEMENT.get(facility_id, False)
        self.get_logger().warn(
            f'No judgment from equipment_db_node for {facility_id} '
            f'within {self.JUDGMENT_TIMEOUT}s. Using local fallback: '
            f'need_replacement={fallback}')
        return fallback

    def publish_replacement_result(self, facility, success, reason='', part_id=None):
        result = {
            'facility_id': facility['facility_id'],
            'robot_id': self.robot_id,
            'result': 'SUCCESS' if success else 'FAILED',
            'part_id': part_id,
            'reason': reason,
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

    # ------------------------------------------------------------------
    # FSM 메인 루프
    # ------------------------------------------------------------------
    def run_fsm(self):
        handlers = {
            State.IDLE: self.on_idle,
            State.PATROLLING: self.on_patrolling,
            State.INSPECTING: self.on_inspecting,
            State.RETURNING_TO_BASE: self.on_returning_to_base,
            State.ARRIVED_BASE: self.on_arrived_base,
            State.PART_CHECK: self.on_part_check,
            State.MOVING_TO_FACILITY: self.on_moving_to_facility,
            State.REPLACING: self.on_replacing,
            State.EMERGENCY_RESPONSE: self.on_emergency_response,
        }

        while not self.fsm_stop.is_set():
            # 긴급 이벤트 최우선 선점 (지침서: 긴급 미션 우선, 기존 미션 중단 저장 후 재개)
            if self.emergency_pending() and self.state not in (
                    State.IDLE, State.EMERGENCY_RESPONSE):
                self.resume_state = self.resume_point_for(self.state)
                self.set_state(State.EMERGENCY_RESPONSE,
                               f'resume={self.resume_state.name}')
                continue

            next_state = handlers[self.state]()
            if next_state is None:
                break  # 종료 요청
            if next_state is not self.state:
                self.set_state(next_state)

        self.get_logger().info('FSM loop terminated.')

    @staticmethod
    def resume_point_for(state):
        """ 긴급 대응 후 복귀 지점.
        순찰 흐름은 현재 설비부터 다시(PATROLLING), 교체 흐름은 큐 항목이
        완료 전까지 큐에 남아 있으므로 베이스 복귀부터 재시작하면 안전하다. """
        if state in (State.PATROLLING, State.INSPECTING):
            return State.PATROLLING
        return State.RETURNING_TO_BASE

    # ------------------------------------------------------------------
    # 상태별 핸들러 (반환값 = 다음 상태, None = 종료)
    # ------------------------------------------------------------------
    def on_idle(self):
        """ 시나리오 1: 시스템 시작. 도킹 확인 -> 초기 포즈 -> Nav2 -> 언도킹 """
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()
        initial_pose = self.navigator.getPoseStamped(
            self.INITIAL_POSE_POSITION, self.INITIAL_POSE_DIRECTION)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()
        self.get_logger().info('Startup complete. Beginning patrol.')
        return State.PATROLLING

    def on_patrolling(self):
        """ 시나리오 2: 다음 설비로 이동. 한 바퀴 완료 시 교체 큐 처리 분기 """
        if self.facility_idx >= len(self.FACILITIES):
            # 한 바퀴 순찰 완료? (지침서 다이아몬드 분기)
            self.facility_idx = 0
            if self.replace_queue:
                queued = [f['facility_id'] for f in self.replace_queue]
                self.get_logger().info(f'Patrol lap done. Replacement queue: {queued}')
                return State.RETURNING_TO_BASE
            self.get_logger().info('Patrol lap done. Queue empty -> next lap.')

        facility = self.FACILITIES[self.facility_idx]
        res = self.navigate_to(facility['waypoint'], facility['direction'],
                               facility['facility_id'])
        if res == 'ok':
            return State.INSPECTING
        if res == 'preempted':
            return State.PATROLLING  # 루프 상단에서 EMERGENCY 로 전환됨
        if res == 'stopped':
            return None
        # 이동 실패: 해당 설비 건너뛰고 순찰 계속 (지침서 예외: 이동 실패)
        self.get_logger().error(
            f"Navigation to {facility['facility_id']} failed. Skipping.")
        self.facility_idx += 1
        return State.PATROLLING

    def on_inspecting(self):
        """ 시나리오 3/4: ArUco 인식 -> ticket_info -> 판정 -> CHECKED/큐 등록 """
        facility = self.FACILITIES[self.facility_idx]
        fid = facility['facility_id']

        if not self.wait_for_marker(facility['marker_id'], self.DETECT_WINDOW):
            if self.emergency_pending():
                return State.INSPECTING  # 루프 상단에서 EMERGENCY 로 전환됨
            # 지침서 예외: ArUco 미검출 -> FAILED_CHECK 기록 후 다음 설비로
            self.get_logger().warn(
                f'[{fid}] Marker {facility["marker_id"]} not found in '
                f'{self.DETECT_WINDOW}s. FAILED_CHECK, moving on.')
            self.set_state(State.INSPECTING, f'{fid}: FAILED_CHECK')
            self.facility_idx += 1
            return State.PATROLLING

        self.get_logger().info(f'[{fid}] Marker confirmed. Sending ticket_info.')
        self.publish_ticket(facility)
        need_replacement = self.wait_judgment(fid)

        if need_replacement:
            # duplicate_guard: 동일 설비 중복 등록 방지
            if all(f['facility_id'] != fid for f in self.replace_queue):
                self.replace_queue.append(facility)
                self.set_state(State.INSPECTING, f'{fid}: NEED_REPLACEMENT (queued)')
            else:
                self.get_logger().info(f'[{fid}] Already in replacement queue.')
        else:
            self.set_state(State.INSPECTING, f'{fid}: CHECKED')

        self.facility_idx += 1
        return State.PATROLLING

    def on_returning_to_base(self):
        """ 시나리오 5: 베이스 복귀 """
        res = self.navigate_to(self.BASE_POSE[0], self.BASE_POSE[1], 'BASE')
        if res == 'ok':
            return State.ARRIVED_BASE
        if res == 'preempted':
            return State.RETURNING_TO_BASE
        if res == 'stopped':
            return None
        self.get_logger().error('Return to base failed. Retrying in 2s...')
        time.sleep(2.0)
        return State.RETURNING_TO_BASE

    def on_arrived_base(self):
        """ 베이스 도착: 남은 교체 큐가 있으면 물품 CHECK, 없으면 순찰 재개 """
        if self.replace_queue:
            return State.PART_CHECK
        self.get_logger().info('All replacements handled. Resuming patrol.')
        return State.PATROLLING

    def on_part_check(self):
        """ 시나리오 5: inventory CHECK(PART_CHECKED) -> 적재(PART_LOADED).
        큐 항목은 여기서 꺼내지 않고(peek) 성공/실패 확정 시에만 제거한다.
        (긴급 선점으로 흐름이 끊겨도 항목이 유실되지 않도록) """
        facility = self.replace_queue[0]
        part_type = facility['part_type']
        qty = self.stock.get(part_type, 0)

        if qty <= 0:
            # 지침서 예외: 재고 부족 -> PART_LOADED 로 넘어가지 않고 FAILED 처리
            self.get_logger().error(
                f"[{facility['facility_id']}] No stock for {part_type}. FAILED.")
            self.publish_replacement_result(facility, False, reason='NO_STOCK')
            self.replace_queue.pop(0)
            return State.ARRIVED_BASE  # 다음 큐 항목/순찰 재개 재분기

        self.set_state(State.PART_CHECK,
                       f"PART_CHECKED: {part_type} x{qty} for {facility['facility_id']}")
        time.sleep(self.LOAD_DURATION)  # 베이스 적재 수행 가정
        self.set_state(State.PART_CHECK, f'PART_LOADED: {part_type}')
        return State.MOVING_TO_FACILITY

    def on_moving_to_facility(self):
        """ 시나리오 6: 교체 대상 설비로 이동 """
        facility = self.replace_queue[0]
        res = self.navigate_to(facility['waypoint'], facility['direction'],
                               f"{facility['facility_id']} (replace)")
        if res == 'ok':
            return State.REPLACING
        if res == 'preempted':
            return State.MOVING_TO_FACILITY
        if res == 'stopped':
            return None
        self.get_logger().error(
            f"Navigation to {facility['facility_id']} failed. FAILED.")
        self.publish_replacement_result(facility, False, reason='NAV_FAIL')
        self.replace_queue.pop(0)
        return State.RETURNING_TO_BASE

    def on_replacing(self):
        """ 시나리오 6: 설비 marker_id 재확인(정렬) -> maintenance_actuator 교체
        -> replacement_result 발행 -> 재고 차감 """
        facility = self.replace_queue[0]
        fid = facility['facility_id']

        if not self.wait_for_marker(facility['marker_id'], self.DETECT_WINDOW):
            if self.emergency_pending():
                return State.REPLACING
            self.get_logger().error(f'[{fid}] Marker re-check failed. FAILED.')
            self.publish_replacement_result(facility, False, reason='MARKER_NOT_FOUND')
            self.replace_queue.pop(0)
            return State.RETURNING_TO_BASE

        part_type = facility['part_type']
        part_id = f'{part_type}-{int(time.time())}'
        self.set_state(State.REPLACING, f'{fid}: actuator running ({part_id})')
        time.sleep(self.REPLACE_DURATION)  # maintenance_actuator 교체 수행 가정

        self.stock[part_type] = self.stock.get(part_type, 0) - 1
        self.publish_replacement_result(facility, True, part_id=part_id)
        self.replace_queue.pop(0)
        self.set_state(State.REPLACING,
                       f'{fid}: REPLACED ({part_id}), stock {part_type}={self.stock[part_type]}')
        return State.RETURNING_TO_BASE

    def on_emergency_response(self):
        """ 시나리오 7: 긴급 지점 출동 -> 현장 대기 -> 저장된 상태로 복귀 """
        with self.lock:
            goal = self.emergency_goal
        if goal is None:
            return self.resume_state

        res = self.navigate_to(list(goal), TurtleBot4Directions.NORTH, 'EMERGENCY')
        if res == 'stopped':
            return None
        if res == 'failed':
            self.get_logger().error('Emergency navigation failed.')
        else:
            self.get_logger().info(
                f'Arrived at emergency point. Holding {self.EMERGENCY_HOLD}s.')
            time.sleep(self.EMERGENCY_HOLD)

        with self.lock:
            self.emergency_goal = None
        self.get_logger().info(f'Emergency handled. Resuming: {self.resume_state.name}')
        return self.resume_state

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.fsm_stop.set()
        # navigate_to 폴링 루프가 fsm_stop 을 보고 cancelTask 하지만,
        # waitUntilNav2Active 등 취소 불가 구간 대비 여기서도 한 번 취소
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        self.fsm_thread.join(timeout=5.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = PatrolFSMNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
