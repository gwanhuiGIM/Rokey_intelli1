#!/usr/bin/env python3
"""
웹캠(cv2.VideoCapture) 입력으로 ArUco 마커를 탐지하는 ROS2 노드

- 검출 결과(/patrol/marker_detection)를 JSON 문자열로 발행
  -> 이후 점검 기록 노드(db_manager)가 구독해 inspection_log 기록에 활용
- 카메라 캘리브레이션(yaml) 제공 시 마커까지의 거리/포즈(tvec, rvec)도 계산
- 화면에 검출 결과 오버레이 표시 (q 종료)

파라미터:
  camera_device  : V4L2 장치 (기본: /dev/video0)
  width / height : 캡처 해상도 (기본 640x480)
  fps            : 캡처 FPS (기본 30)
  aruco_dict     : 사전 이름 (기본 DICT_4X4_50)
  marker_length  : 실제 마커 한 변 길이[m] (기본 0.10) — 포즈 추정용
  calib_file     : 캘리브레이션 yaml 경로 ('' 이면 포즈 추정 생략, ID/픽셀좌표만)
  publish_topic  : 검출 결과 토픽 (기본 /patrol/marker_detection)
  show_window    : OpenCV 창 표시 여부 (기본 true)

실행 예:
  python3 aruco_detect_node.py --ros-args \
      -p camera_device:=/dev/video2 \
      -p aruco_dict:=DICT_4X4_50 \
      -p marker_length:=0.10 \
      -p calib_file:=/home/kimkh/calib/webcam.yaml

캘리브레이션 yaml 형식 (cv2.FileStorage 호환):
  %YAML:1.0
  camera_matrix: !!opencv-matrix
    rows: 3
    cols: 3
    dt: d
    data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  dist_coeffs: !!opencv-matrix
    rows: 1
    cols: 5
    dt: d
    data: [k1, k2, p1, p2, k3]
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

# cv2.aruco 사전 이름 -> 상수 매핑
ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')

        # ---- 파라미터 ----
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_length', 0.10)
        self.declare_parameter('calib_file', '')
        self.declare_parameter('publish_topic', '/patrol/marker_detection')
        self.declare_parameter('show_window', True)

        gp = lambda n: self.get_parameter(n).value
        self.camera_device = gp('camera_device')
        self.width = gp('width')
        self.height = gp('height')
        self.fps = gp('fps')
        dict_name = gp('aruco_dict')
        self.marker_length = gp('marker_length')
        calib_file = gp('calib_file')
        publish_topic = gp('publish_topic')
        self.show_window = gp('show_window')

        # ---- ArUco 검출기 ----
        if dict_name not in ARUCO_DICTS:
            raise ValueError(f"지원하지 않는 사전: {dict_name} (지원: {list(ARUCO_DICTS)})")
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # ---- 캘리브레이션 (선택) ----
        self.camera_matrix = None
        self.dist_coeffs = None
        if calib_file:
            fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
            if fs.isOpened():
                self.camera_matrix = fs.getNode('camera_matrix').mat()
                self.dist_coeffs = fs.getNode('dist_coeffs').mat()
                fs.release()
                self.get_logger().info(f"캘리브레이션 로드 완료: {calib_file}")
            else:
                self.get_logger().warn(f"캘리브레이션 파일 열기 실패: {calib_file} — 포즈 추정 생략")

        # ---- 웹캠 ----
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            raise RuntimeError(f"카메라를 열 수 없습니다: {self.camera_device}")

        self.get_logger().info(
            f"웹캠 연결: {self.camera_device} "
            f"({int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))})"
        )

        # ---- 발행자 ----
        # 검출 이벤트는 유실되면 안 되므로 RELIABLE (센서 스트림과 구분)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.detection_pub = self.create_publisher(String, publish_topic, qos_profile)

        # 웹캠 폴링 타이머 (fps에 맞춰)
        self.timer = self.create_timer(1.0 / self.fps, self.capture_and_detect)

    def capture_and_detect(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("웹캠 프레임 읽기 실패")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self.detector.detectMarkers(gray)

        detections = []
        if ids is not None:
            # 포즈 추정 (캘리브레이션 있을 때만)
            rvecs = tvecs = None
            if self.camera_matrix is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length,
                    self.camera_matrix, self.dist_coeffs
                )

            for i, (marker_corners, marker_id) in enumerate(zip(corners, ids.flatten())):
                center = marker_corners[0].mean(axis=0)
                det = {
                    "id": int(marker_id),
                    "center_px": [float(center[0]), float(center[1])],
                }
                if tvecs is not None:
                    tvec = tvecs[i][0]
                    det["distance_m"] = float(np.linalg.norm(tvec))
                    det["tvec"] = [float(v) for v in tvec]
                    det["rvec"] = [float(v) for v in rvecs[i][0]]
                detections.append(det)

                # 오버레이
                if self.show_window and tvecs is not None:
                    cv2.drawFrameAxes(
                        frame, self.camera_matrix, self.dist_coeffs,
                        rvecs[i], tvecs[i], self.marker_length * 0.5
                    )

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # 검출 결과 발행
            msg = String()
            msg.data = json.dumps({
                "stamp": time.time(),
                "detections": detections,
            })
            self.detection_pub.publish(msg)

        if self.show_window:
            cv2.imshow("ArUco Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("q 입력 — 종료")
                rclpy.shutdown()

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()