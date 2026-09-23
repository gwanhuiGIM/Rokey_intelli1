#!/usr/bin/env python3
"""
AMR 구동 노드 (로봇 측, PC1/PC2 의 patrol_navigation + aruco_detect 역할).

FSM(관제) 노드와 분리된 '실행 전용' 노드:
  - /fleet/mission_request 에서 자기 robot_id 의 미션을 받아 수행만 한다.
    (PATROL: 소화기 waypoint 한 바퀴 점검 / DISPATCH: 지정 좌표 긴급 출동 /
     RETURN_TO_BASE: 베이스 복귀)
  - 판단(다음 미션, 출동 로봇 선정, 점검 집계)은 전부 fleet_fsm 노드가 한다.
  - 새 미션이 오면 진행 중인 미션을 취소하고 즉시 전환한다 (관제가 두뇌, 로봇은 손발).

발행:
  /fleet/robot_status  (String/JSON) 상태 전이마다: {robot_id, state, pose, mission_id}
  /fleet/amr_event     (String/JSON) 점검/출동 이벤트:
      {type: INSPECTION, facility_id, marker_id, result: CHECKED|FAILED_CHECK, ...}
      {type: PATROL_DONE, checked, failed, total}
      {type: DISPATCH_ARRIVED / DISPATCH_DONE, goal}
  <ns>/patrol/rgb_processed  (Image) ArUco 오버레이 디버그 영상

구독:
  /fleet/mission_request (String/JSON): {robot_id, mission_type, goal?, mission_id}
  <ns>/oakd/rgb/image_raw/compressed, <ns>/oakd/rgb/camera_info
"""

import json
import threading
import time

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
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class AmrAgentNode(Node):

    # === 시작/베이스 자세 ===
    INITIAL_POSE_POSITION = [0.0, 0.0]
    INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH
    BASE_POSE = ([0.0, 0.0], TurtleBot4Directions.NORTH)

    # === 소화기(설비) waypoint 목록: 실제 맵 좌표/마커 ID 로 교체할 것 ===
    FACILITIES = [
        {'facility_id': 'EXT_01', 'waypoint': [-0.02, -1.39],
         'direction': TurtleBot4Directions.NORTH, 'marker_id': 0},
        {'facility_id': 'EXT_02', 'waypoint': [-2.77, -1.29],
         'direction': TurtleBot4Directions.SOUTH, 'marker_id': 1},
        {'facility_id': 'EXT_03', 'waypoint': [-0.29, -0.22],
         'direction': TurtleBot4Directions.EAST, 'marker_id': 2},
    ]

    # === ArUco 설정 ===
    ARUCO_DICT = cv2.aruco.DICT_5X5_50
    DETECT_WINDOW = 5.0     # 소화기 앞 마커 탐색 제한 시간(초)
    DETECT_MIN_HITS = 3     # 연속 N 프레임 감지 시 확정 (오검출 방지)

    DISPATCH_HOLD = 5.0     # 출동 지점 도착 후 현장 대기(초)

    def __init__(self):
        super().__init__('amr_agent_node')

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # get_namespace() 는 네임스페이스가 없으면 '/' (루트) 를 반환한다.
        # 그대로 f'{ns}/...' 에 쓰면 '//oakd/...' 처럼 슬래시가 중복되어
        # rclpy 가 토픽명을 거부하므로 루트일 때는 빈 문자열로 정규화한다.
        ns = self.get_namespace()
        if ns == '/':
            ns = ''
        self.robot_id = ns.strip('/') or 'robot1'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'

        # 공유 상태 (lock 보호)
        self.rgb_seq = 0
        self.last_aruco_ids = None
        self.pending_mission = None   # 관제에서 새로 내려온 미션 (최신 1개만 유지)

        self.state = 'STARTING'
        self.current_mission_id = None
        # 도킹 여부는 worker 스레드에서 dock()/undock() 호출 시점에 직접 갱신
        # (navigator 의 is_docked 는 해당 노드를 spin 할 때만 갱신되므로 신뢰 불가)
        self.docked = False

        dictionary = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        self.aruco_detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.status_pub = self.create_publisher(String, '/fleet/robot_status', 10)
        self.event_pub = self.create_publisher(String, '/fleet/amr_event', 10)
        self.rgb_pub = self.create_publisher(ROSImage, f'{ns}/patrol/rgb_processed', 1)

        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, img_qos)
        self.create_subscription(String, '/fleet/mission_request', self.mission_callback, 10)

        self.navigator = TurtleBot4Navigator()

        # 주기 상태 발행 (관제의 로봇 생존 감시 + 최신 pose 제공용 하트비트)
        self.create_timer(1.0, lambda: self.publish_status(self.state))

        self.worker_stop = threading.Event()
        self.worker_thread = threading.Thread(target=self.worker, daemon=True)
        self.worker_thread.start()

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
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
            cv2.putText(overlay, f'{self.robot_id}: {self.state}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header.stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(overlay_msg)
        except Exception as e:
            self.get_logger().error(f'RGB callback failed: {e}')

    def mission_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid mission message: {e}')
            return
        if data.get('robot_id') != self.robot_id:
            return
        with self.lock:
            self.pending_mission = data
        self.get_logger().info(
            f"Mission received: {data.get('mission_type')} "
            f"(id={data.get('mission_id')})")

    # ------------------------------------------------------------------
    # 공용 헬퍼
    # ------------------------------------------------------------------
    def has_pending_mission(self):
        with self.lock:
            return self.pending_mission is not None

    def take_mission(self):
        with self.lock:
            mission = self.pending_mission
            self.pending_mission = None
        return mission

    def get_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=0.5))
            return [round(tf.transform.translation.x, 3),
                    round(tf.transform.translation.y, 3)]
        except Exception:
            return None

    def publish_status(self, state, detail=''):
        self.state = state
        msg = String()
        msg.data = json.dumps({
            'robot_id': self.robot_id,
            'state': state,
            'detail': detail,
            'pose': self.get_robot_xy(),
            'docked': self.docked,
            'mission_id': self.current_mission_id,
            'timestamp': time.time(),
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def publish_event(self, event_type, **fields):
        payload = {'type': event_type, 'robot_id': self.robot_id,
                   'mission_id': self.current_mission_id,
                   'timestamp': time.time()}
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.event_pub.publish(msg)

    def navigate_to(self, position, direction, label):
        """ goToPose + 폴링. 새 미션(선점)/종료 요청 시 즉시 취소.
        반환: 'ok' | 'failed' | 'preempted' | 'stopped' """
        pose = self.navigator.getPoseStamped(position, direction)
        self.get_logger().info(f'Navigating to {label}: {position}')
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            if self.worker_stop.is_set():
                self.navigator.cancelTask()
                return 'stopped'
            if self.has_pending_mission():
                self.navigator.cancelTask()
                return 'preempted'
            time.sleep(0.2)

        result = self.navigator.getResult()
        return 'ok' if result == TaskResult.SUCCEEDED else 'failed'

    def wait_for_marker(self, marker_id, window):
        """ 연속 DETECT_MIN_HITS 프레임 감지 시 True. 선점/종료 시 즉시 중단 """
        start = time.monotonic()
        hits = 0
        last_seq = -1
        while (time.monotonic() - start < window
                and not self.worker_stop.is_set()
                and not self.has_pending_mission()):
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
                    hits = 0
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------------
    # 미션 실행 루프
    # ------------------------------------------------------------------
    def ensure_undocked(self):
        """ 도킹 상태에서 이동 미션을 받으면 먼저 언도킹 """
        if self.docked:
            self.publish_status('UNDOCKING')
            self.navigator.undock()
            self.docked = False

    def worker(self):
        # 시스템 시작: 도킹 확인 -> 초기 포즈 -> Nav2 (언도킹은 첫 미션 때 수행)
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()
        self.docked = True
        self.publish_status('DOCKED', 'startup')
        initial_pose = self.navigator.getPoseStamped(
            self.INITIAL_POSE_POSITION, self.INITIAL_POSE_DIRECTION)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.publish_status('IDLE', 'startup complete (docked)')
        self.get_logger().info('Startup complete. Waiting for missions.')

        while not self.worker_stop.is_set():
            mission = self.take_mission()
            if mission is None:
                time.sleep(0.1)
                continue

            self.current_mission_id = mission.get('mission_id')
            mtype = mission.get('mission_type')
            if mtype == 'PATROL':
                self.do_patrol()
            elif mtype == 'DISPATCH':
                self.do_dispatch(mission)
            elif mtype == 'RETURN_TO_BASE':
                self.do_return_to_base()
            elif mtype == 'RETURN_TO_DOCK':
                self.do_return_to_dock(mission)
            else:
                self.get_logger().warn(f'Unknown mission_type: {mtype}')

            # 선점된 경우 pending_mission 이 이미 차 있으므로 즉시 다음 미션 실행됨
            if not self.has_pending_mission():
                self.current_mission_id = None
                self.publish_status('DOCKED' if self.docked else 'IDLE')

    def do_patrol(self):
        """ 소화기 waypoint 한 바퀴 점검. 각 지점 결과를 이벤트로 발행하고
        완료 시 PATROL_DONE 발행. 다음 바퀴 여부는 관제(fleet_fsm)가 결정 """
        self.ensure_undocked()
        checked, failed = [], []
        for facility in self.FACILITIES:
            fid = facility['facility_id']
            self.publish_status('PATROLLING', f'-> {fid}')
            res = self.navigate_to(facility['waypoint'], facility['direction'], fid)
            if res in ('preempted', 'stopped'):
                self.publish_event('PATROL_ABORTED', checked=checked, failed=failed)
                return
            if res == 'failed':
                self.get_logger().error(f'[{fid}] Navigation failed. Skipping.')
                failed.append(fid)
                self.publish_event('INSPECTION', facility_id=fid,
                                   marker_id=facility['marker_id'],
                                   result='NAV_FAIL')
                continue

            self.publish_status('INSPECTING', fid)
            if self.wait_for_marker(facility['marker_id'], self.DETECT_WINDOW):
                checked.append(fid)
                self.get_logger().info(f'[{fid}] Extinguisher marker CHECKED.')
                self.publish_event('INSPECTION', facility_id=fid,
                                   marker_id=facility['marker_id'],
                                   result='CHECKED', pose=self.get_robot_xy())
            else:
                if self.has_pending_mission() or self.worker_stop.is_set():
                    self.publish_event('PATROL_ABORTED', checked=checked, failed=failed)
                    return
                failed.append(fid)
                self.get_logger().warn(f'[{fid}] Marker NOT found. FAILED_CHECK.')
                self.publish_event('INSPECTION', facility_id=fid,
                                   marker_id=facility['marker_id'],
                                   result='FAILED_CHECK')

        self.publish_event('PATROL_DONE', checked=checked, failed=failed,
                           total=len(self.FACILITIES))
        self.get_logger().info(
            f'Patrol lap done: {len(checked)}/{len(self.FACILITIES)} checked.')

    def do_dispatch(self, mission):
        """ 긴급 출동: 지정 좌표로 이동 -> 도착 보고 -> 현장 대기 -> 완료 보고 """
        goal = mission.get('goal') or {}
        try:
            gx, gy = float(goal['x']), float(goal['y'])
        except Exception:
            self.get_logger().error(f'DISPATCH mission without valid goal: {mission}')
            return

        self.ensure_undocked()
        reason = mission.get('reason', 'EMERGENCY')
        self.publish_status('DISPATCHING', f'{reason} -> ({gx:.2f}, {gy:.2f})')
        res = self.navigate_to([gx, gy], TurtleBot4Directions.NORTH, 'DISPATCH')
        if res in ('preempted', 'stopped'):
            return
        if res == 'failed':
            self.publish_event('DISPATCH_FAILED', goal=goal)
            self.get_logger().error('Dispatch navigation failed.')
            return

        self.publish_event('DISPATCH_ARRIVED', goal=goal)
        self.publish_status('ON_SCENE', f'holding {self.DISPATCH_HOLD}s')
        deadline = time.monotonic() + self.DISPATCH_HOLD
        while time.monotonic() < deadline:
            if self.worker_stop.is_set() or self.has_pending_mission():
                return
            time.sleep(0.1)
        self.publish_event('DISPATCH_DONE', goal=goal)

    def do_return_to_base(self):
        self.ensure_undocked()
        self.publish_status('RETURNING_TO_BASE')
        res = self.navigate_to(self.BASE_POSE[0], self.BASE_POSE[1], 'BASE')
        if res == 'ok':
            self.publish_event('ARRIVED_BASE')

    def do_return_to_dock(self, mission):
        """ 배터리부족 등으로 도크 복귀 + 도킹(충전 시작) """
        reason = mission.get('reason', '')
        self.publish_status('RETURNING_TO_DOCK', reason)
        res = self.navigate_to(self.BASE_POSE[0], self.BASE_POSE[1], 'DOCK')
        if res in ('preempted', 'stopped'):
            return
        if res == 'failed':
            self.get_logger().error('Return to dock navigation failed.')
            # 도크 근처 이동 실패여도 도킹은 시도해 본다 (Nav2 goal 실패 != 도킹 불가)
        self.publish_status('DOCKING')
        self.navigator.dock()
        self.docked = True
        self.publish_event('DOCKED_AT_BASE', reason=reason)
        self.publish_status('DOCKED', reason)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.worker_stop.set()
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        self.worker_thread.join(timeout=5.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = AmrAgentNode()
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
