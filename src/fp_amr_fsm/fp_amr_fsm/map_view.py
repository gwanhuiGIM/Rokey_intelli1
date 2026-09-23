"""지도 로드 + OpenCV 관제 화면 (fleet_fsm.py 에서 분리한 표시 전용 모듈).

노드 로직과 분리하기 위해, 그릴 데이터는 snapshot_fn() 콜백으로 매 프레임
받아온다 (노드가 lock 안에서 dict 로 복사해 반환). 로직 변경 없음.
"""

import os
import threading

import cv2
import numpy as np
import yaml

from .robot_context import STATE_COLOR


class MapView:

    def __init__(self, map_pgm, map_yaml, viz_scale, panel_w, logger):
        self.map_pgm = map_pgm
        self.map_yaml = map_yaml
        self.viz_scale = viz_scale
        self.panel_w = panel_w
        self.logger = logger
        self.stop_event = threading.Event()
        self.thread = None
        self.load_map()

    # ------------------------------------------------------------------
    # 지도 로드 / 좌표 변환
    # ------------------------------------------------------------------
    def load_map(self):
        self.map_img = None
        self.map_res = 0.05
        self.map_origin = (0.0, 0.0)
        if os.path.exists(self.map_pgm) and os.path.exists(self.map_yaml):
            img = cv2.imread(self.map_pgm, cv2.IMREAD_GRAYSCALE)
            with open(self.map_yaml, 'r') as f:
                meta = yaml.safe_load(f)
            if img is not None:
                self.map_img = img
                self.map_res = float(meta['resolution'])
                self.map_origin = (float(meta['origin'][0]), float(meta['origin'][1]))
                self.logger.info(
                    f'Map loaded: {self.map_pgm} {img.shape[1]}x{img.shape[0]}px, '
                    f'res={self.map_res}, origin={self.map_origin}')
                return
        self.logger.warn(
            f'Map not found ({self.map_pgm}). Using blank canvas.')
        self.map_img = np.full((240, 240), 205, dtype=np.uint8)
        self.map_origin = (-6.0, -6.0)

    def world_to_px(self, x, y):
        """ map 좌표(m) -> 확대된 지도 이미지 픽셀.
        지도 범위 밖 좌표는 가장자리에 클램프 (로봇이 화면에서 사라지지 않도록) """
        h, w = self.map_img.shape[:2]
        px = (x - self.map_origin[0]) / self.map_res
        py = h - 1 - (y - self.map_origin[1]) / self.map_res
        px = min(max(px, 0), w - 1)
        py = min(max(py, 0), h - 1)
        return int(px * self.viz_scale), int(py * self.viz_scale)

    # ------------------------------------------------------------------
    # 렌더링
    # ------------------------------------------------------------------
    def render_frame(self, snap):
        """ map.pgm 위에 로봇/상황/이벤트 지점을 그린 프레임 생성.
        snap: {'situation', 'emg_pts', 'helmet_pts', 'robots'} (노드가 lock
        안에서 복사한 스냅샷) """
        base = cv2.resize(
            self.map_img, None, fx=self.viz_scale, fy=self.viz_scale,
            interpolation=cv2.INTER_NEAREST)
        canvas = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)

        situation = snap['situation']
        emg_pts = snap['emg_pts']
        helmet_pts = snap['helmet_pts']
        robots = snap['robots']

        # 출동 목표 지점 + 로봇-목표 연결선
        for rb in robots:
            if rb['goal'] is not None:
                gp = self.world_to_px(*rb['goal'])
                color = (60, 60, 255) if rb['reason'] == 'EMERGENCY' else (0, 165, 255)
                cv2.drawMarker(canvas, gp, color, cv2.MARKER_TILTED_CROSS, 22, 3)
                if rb['pose'] is not None:
                    cv2.line(canvas, self.world_to_px(*rb['pose']), gp, color, 1,
                             cv2.LINE_AA)

        # 미배정 이벤트 지점
        for x, y in emg_pts:
            cv2.drawMarker(canvas, self.world_to_px(x, y), (60, 60, 255),
                           cv2.MARKER_TILTED_CROSS, 22, 3)
        for x, y in helmet_pts:
            cv2.circle(canvas, self.world_to_px(x, y), 10, (0, 165, 255), 2)

        # 로봇
        for rb in robots:
            if rb['pose'] is None:
                continue
            p = self.world_to_px(*rb['pose'])
            color = STATE_COLOR.get(rb['state'], (255, 255, 255))
            cv2.circle(canvas, p, 12, color, -1)
            cv2.circle(canvas, p, 12, (30, 30, 30), 2)
            label = rb['id'].replace('robot', 'R')
            cv2.putText(canvas, label, (p[0] - 12, p[1] - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 3)
            cv2.putText(canvas, label, (p[0] - 12, p[1] - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 상황 배너 (지도 상단)
        banner_color = (0, 0, 200) if situation == 'EMERGENCY' else (0, 130, 0)
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), banner_color, -1)
        banner = ('EMERGENCY - dispatching to webcam-detected position'
                  if situation == 'EMERGENCY' else 'NORMAL - routine patrol')
        cv2.putText(canvas, banner, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # 우측 상태 패널
        panel = np.full((canvas.shape[0], self.panel_w, 3), 30, dtype=np.uint8)
        y = 40
        cv2.putText(panel, 'FLEET MONITOR', (14, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        y += 40
        for rb in robots:
            color = STATE_COLOR.get(rb['state'], (255, 255, 255))
            batt = f"{rb['battery']*100:.0f}%" if rb['battery'] is not None else '?'
            pose = (f"({rb['pose'][0]:.2f}, {rb['pose'][1]:.2f})"
                    if rb['pose'] else '(?, ?)')
            cv2.circle(panel, (24, y - 6), 8, color, -1)
            cv2.putText(panel, f"{rb['id']}  batt {batt}", (42, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y += 26
            cv2.putText(panel, rb['state'], (42, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            y += 24
            cv2.putText(panel, pose, (42, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
            y += 34
        y += 6
        cv2.putText(panel, f'emergency queue: {len(emg_pts)}', (14, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (60, 60, 255), 2)
        y += 26
        cv2.putText(panel, f'helmet queue: {len(helmet_pts)}', (14, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
        y += 40
        # 범례
        for state in ('NAVIGATING', 'DISPATCHING_EMERGENCY',
                      'DISPATCHING_HELMET', 'RETURNING_LOW_BATTERY', 'CHARGING',
                      'IDLE', 'NO_LOCALIZATION', 'NO_NAV2', 'OFFLINE'):
            cv2.circle(panel, (24, y - 5), 6, STATE_COLOR[state], -1)
            cv2.putText(panel, state, (40, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
            y += 22

        return np.hstack([canvas, panel])

    # ------------------------------------------------------------------
    # 표시 스레드
    # ------------------------------------------------------------------
    def start(self, snapshot_fn):
        """ 지도 창 스레드 시작. snapshot_fn() 은 render_frame 이 그릴
        스냅샷 dict 를 반환해야 한다 (노드 쪽에서 lock 처리). """
        self.thread = threading.Thread(
            target=self._gui_loop, args=(snapshot_fn,), daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)

    def _gui_loop(self, snapshot_fn):
        # 중요: DISPLAY 가 없으면 cv2.namedWindow() 진입 자체를 막아야 한다.
        # 이 OpenCV 빌드의 Qt 백엔드는 플러그인 로드 실패 시 파이썬 예외가 아니라
        # Qt 내부에서 qFatal()/abort() 를 호출해 프로세스 전체가 죽는다
        # (SIGABRT). try/except 로는 잡히지 않으므로, SSH/systemd 등 headless
        # 환경에서 관제 노드 전체가 함께 죽는 것을 막기 위해 사전 차단한다.
        if not os.environ.get('DISPLAY'):
            self.logger.warn(
                'DISPLAY not set. Map window disabled - terminal monitoring only.')
            return
        window = 'Fleet Monitor (map)'
        try:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        except Exception as e:
            self.logger.warn(
                f'Map window unavailable ({e}). Terminal monitoring only.')
            return
        while not self.stop_event.is_set():
            try:
                cv2.imshow(window, self.render_frame(snapshot_fn()))
                if cv2.waitKey(200) & 0xFF == ord('q'):
                    break
            except Exception as e:
                self.logger.warn(
                    f'Map window failed ({e}). Terminal monitoring only.')
                return
        cv2.destroyAllWindows()
