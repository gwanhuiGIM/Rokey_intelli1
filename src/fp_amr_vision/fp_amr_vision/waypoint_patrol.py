#!/usr/bin/env python3
"""
Waypoint 순찰 + 소화기 ArUco 마커 인식 노드.

동작 개요:
  1. 등록된 waypoint 목록을 순서대로 순찰 (Nav2)
  2. 각 waypoint 도착 시 일정 시간 동안 카메라 영상에서 ArUco 마커를 탐색
  3. 소화기에 부착된 마커(FIRE_EXTINGUISHER_MARKER_ID)가 보이면
     - 로그 출력
     - 결과를 토픽으로 발행 (JSON 문자열)
  4. 다음 waypoint로 이동 (전체 waypoint 반복 순찰)

[YOLO 대신 ArUco를 쓰는 이유]
move_camera.py의 YOLO 방식은 '차량처럼 모양이 다양한 대상'을 학습으로
구분하기 위한 것이었다. 반면 이번 대상은 "소화기 위치에 고정 부착된
표식(마크)을 인식"하는 것이므로 요구사항이 다르다:
  - 학습 데이터/엔진 파일이 필요 없다 (Archo 마크로 새로 학습시킬 필요 없음)
  - OpenCV cv2.aruco 모듈이 CPU만으로 실시간 검출 가능 (GPU 불필요)
  - 마커 ID로 대상을 구분할 수 있어 다른 표식과 혼동되지 않는다
  - 마커의 4개 코너를 서브픽셀 정확도로 검출하므로, solvePnP로 카메라
    기준 3D 위치(rvec/tvec)를 바로 계산할 수 있다 (Depth 센서 불필요)
따라서 이 노드는 YOLO 엔진을 로드하지 않고 cv2.aruco로 마커를 인식한다.
"""

import json
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point  # noqa: F401 (PointStamped 변환 등록)
from tf2_ros import Buffer, TransformListener
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class WaypointPatrolNode(Node):

    # === 순찰 시작 자세 ===
    INITIAL_POSE_POSITION = [0.0, 0.0]
    INITIAL_POSE_DIRECTION = TurtleBot4Directions.NORTH

    # === 순찰 waypoint 목록: ([x, y], 방향) ===
    # 방향은 TurtleBot4Directions 값(45도 단위) 또는 degrees 숫자 모두 사용 가능
    # 실제 맵 좌표로 교체해서 사용할 것
    WAYPOINTS = [
        ([-0.02, -1.39], TurtleBot4Directions.NORTH),
        ([-2.77, -1.29], TurtleBot4Directions.SOUTH),
        ([-0.29, -0.22], TurtleBot4Directions.EAST),
    ]
    PATROL_LOOP = True  # True: 마지막 waypoint 이후 처음부터 반복 순찰

    # === ArUco 마커 설정 ===
    ARUCO_DICT = cv2.aruco.DICT_5X5_50       # 마커 제작 시 사용한 사전과 일치해야 함
    FIRE_EXTINGUISHER_MARKER_ID = 0          # 소화기에 부착한 마커의 ID
    MARKER_LENGTH_M = 0.10                   # 마커 한 변 길이(m). 실제 출력 크기로 맞출 것
    DETECT_WINDOW = 3.0                      # waypoint 도착 후 마커를 찾는 시간(초)
    DETECT_MIN_HITS = 3                      # 오검출 방지: 최소 연속 감지 프레임 수

    def __init__(self):
        super().__init__('waypoint_patrol_node')

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        ns = self.get_namespace()
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        self.K = None
        self.D = None
        self.rgb_image = None
        self.rgb_seq = 0
        self.last_rgb_time = None
        self.camera_frame = None
        self.last_aruco_ids = None
        self.last_aruco_corners = None

        # ArUco 검출기 (학습/엔진 로딩 없음 -> 노드 시작이 YOLO 대비 훨씬 빠름)
        dictionary = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT)
        parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        # TF2 (카메라 좌표 -> map 좌표 변환용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 결과 발행: 감지 여부/좌표를 JSON 문자열로
        self.result_pub = self.create_publisher(String, f'{ns}/patrol/marker_detection', 10)
        # 디버그용 오버레이 영상 (rqt_image_view 등으로 확인)
        self.rgb_pub = self.create_publisher(ROSImage, f'{ns}/patrol/rgb_processed', 1)

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)

        img_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, img_qos)

        self.navigator = TurtleBot4Navigator()

        # 순찰은 순차적/차단(blocking) 로직이라 별도 스레드에서 실행하고,
        # 이 노드는 실행기(executor)에서 카메라 콜백만 처리한다.
        self.patrol_stop = threading.Event()
        self.patrol_thread = threading.Thread(target=self.run_patrol, daemon=True)
        self.patrol_thread.start()

    # ------------------------------------------------------------------
    # 카메라 콜백
    # ------------------------------------------------------------------
    def camera_info_callback(self, msg):
        with self.lock:
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)

    def rgb_callback(self, rgb_msg):
        try:
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb is None or rgb.size == 0:
                return

            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)

            with self.lock:
                self.rgb_image = rgb
                self.rgb_seq += 1
                self.last_rgb_time = time.monotonic()
                self.camera_frame = rgb_msg.header.frame_id
                self.last_aruco_ids = ids
                self.last_aruco_corners = corners

            overlay = rgb.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(overlay, corners, ids)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header.stamp = self.get_clock().now().to_msg()
            self.rgb_pub.publish(overlay_msg)
        except Exception as e:
            self.get_logger().error(f'RGB callback failed: {e}')

    # ------------------------------------------------------------------
    # 마커 위치 추정 (solvePnP: 마커 4코너 + 카메라 내부파라미터 -> 카메라 기준 3D 위치)
    # ------------------------------------------------------------------
    def estimate_marker_map_point(self, corner_pts, frame_id):
        with self.lock:
            K = self.K
            D = self.D
        if K is None:
            return None

        half = self.MARKER_LENGTH_M / 2.0
        # 마커 코너 순서(top-left, top-right, bottom-right, bottom-left)에 대응하는
        # 마커 로컬 좌표계 3D 점 (마커 중심이 원점, Z=0 평면)
        obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        img_points = corner_pts.reshape(4, 2).astype(np.float32)

        # IPPE_SQUARE: 알려진 크기의 정사각형 평면 마커 전용 풀이 방식.
        # 기본 반복법(SOLVEPNP_ITERATIVE)보다 마커 정면-후면 자세 모호성에 덜 취약함
        ok, _rvec, tvec = cv2.solvePnP(
            obj_points, img_points, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None

        pt_camera = PointStamped()
        pt_camera.header.stamp = Time().to_msg()
        pt_camera.header.frame_id = frame_id
        pt_camera.point.x = float(tvec[0])
        pt_camera.point.y = float(tvec[1])
        pt_camera.point.z = float(tvec[2])

        try:
            pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return None

        return (pt_map.point.x, pt_map.point.y)

    # ------------------------------------------------------------------
    # waypoint 도착 후 마커 탐색
    # ------------------------------------------------------------------
    def check_marker_at_waypoint(self, idx, position):
        self.get_logger().info(
            f'[Waypoint {idx}] Searching for fire-extinguisher marker '
            f'(id={self.FIRE_EXTINGUISHER_MARKER_ID}) for up to {self.DETECT_WINDOW:.1f}s...'
        )

        start = time.monotonic()
        hits = 0
        last_seq = -1
        found_map_point = None

        while (time.monotonic() - start < self.DETECT_WINDOW
                and not self.patrol_stop.is_set()):
            with self.lock:
                seq = self.rgb_seq
                ids = self.last_aruco_ids
                corners = self.last_aruco_corners
                frame_id = self.camera_frame

            if seq != last_seq and frame_id:
                last_seq = seq
                id_list = ids.flatten().tolist() if ids is not None else []
                if self.FIRE_EXTINGUISHER_MARKER_ID in id_list:
                    hits += 1
                    m_idx = id_list.index(self.FIRE_EXTINGUISHER_MARKER_ID)
                    pt = self.estimate_marker_map_point(corners[m_idx], frame_id)
                    if pt is not None:
                        found_map_point = pt
                    if hits >= self.DETECT_MIN_HITS:
                        break
                else:
                    # 이번 프레임엔 안 보임 -> 연속 감지 스트릭 초기화
                    hits = 0
                    found_map_point = None
            time.sleep(0.05)

        found = hits >= self.DETECT_MIN_HITS
        result = {
            'waypoint_index': idx,
            'waypoint_xy': position,
            'marker_id': self.FIRE_EXTINGUISHER_MARKER_ID,
            'found': found,
            'hits': hits,
        }

        if found and found_map_point is not None:
            result['map_x'] = round(found_map_point[0], 3)
            result['map_y'] = round(found_map_point[1], 3)
            self.get_logger().info(
                f"[Waypoint {idx}] Fire extinguisher marker FOUND "
                f"(id={self.FIRE_EXTINGUISHER_MARKER_ID}) at "
                f"map=({found_map_point[0]:.2f}, {found_map_point[1]:.2f})"
            )
        elif found:
            self.get_logger().info(
                f"[Waypoint {idx}] Fire extinguisher marker FOUND but position estimation failed."
            )
        else:
            self.get_logger().warn(
                f"[Waypoint {idx}] Fire extinguisher marker NOT found within "
                f"{self.DETECT_WINDOW:.1f}s (hits={hits})."
            )

        msg = String()
        msg.data = json.dumps(result, ensure_ascii=False)
        self.result_pub.publish(msg)

    # ------------------------------------------------------------------
    # 순찰 스레드
    # ------------------------------------------------------------------
    def run_patrol(self):
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped(
            self.INITIAL_POSE_POSITION, self.INITIAL_POSE_DIRECTION)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        self.get_logger().info('Starting waypoint patrol.')
        while not self.patrol_stop.is_set():
            for idx, (position, direction) in enumerate(self.WAYPOINTS):
                if self.patrol_stop.is_set():
                    break
                pose = self.navigator.getPoseStamped(position, direction)
                self.get_logger().info(f'Heading to waypoint {idx}: {position}')
                self.navigator.startToPose(pose)
                self.check_marker_at_waypoint(idx, position)

            if not self.PATROL_LOOP:
                break

        self.get_logger().info('Patrol finished.')

    def destroy_node(self):
        self.patrol_stop.set()
        # startToPose()는 patrol_stop을 보지 않고 Nav2 목표가 끝날 때까지 블로킹되므로,
        # 진행 중인 목표를 직접 취소해야 patrol_thread가 즉시 빠져나와 join이 끝난다.
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        self.patrol_thread.join(timeout=5.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = WaypointPatrolNode()
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
