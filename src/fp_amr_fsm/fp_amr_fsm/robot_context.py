"""로봇 1대의 관제 상태(RobotContext)와 공용 상수/QoS.

fleet_fsm.py 에서 분리한 모듈. 로직 변경 없음.
"""

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

# 명령/상태 토픽용 QoS: RELIABLE + TRANSIENT_LOCAL(latched) depth 1.
# - RELIABLE: DDS 레벨 재전송으로 유실 방지 (best-effort 는 한 번 놓치면 끝)
# - TRANSIENT_LOCAL: publish 시점에 아직 discovery/매칭이 끝나지 않은
#   구독자(막 뜬 로봇 노드 등)에게도 마지막 메시지 1개를 보관했다가 전달.
#   "한 번만 보내면 가끔 수신 안 되는" 문제의 주원인이 이 discovery 경합이다.
# 주의: 양쪽(발행/구독) 모두 이 QoS 를 써야 매칭된다. 터미널 테스트 시:
#   ros2 topic pub -1 --qos-reliability reliable \
#     --qos-durability transient_local <topic> <type> <msg>
COMMAND_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


# 터미널 표기용 한국어 상태명
# NAVIGATING 은 raw 구동 상태로, patrol_fire.py 의 자체 순찰 이동도 포함한다
# (fleet_fsm 은 순찰을 명령하지 않으므로 별도 PATROLLING 상태를 두지 않는다).
KR_STATE = {
    'OFFLINE': '오프라인',
    'NO_LOCALIZATION': '측위 미실행',
    'NO_NAV2': 'Nav2 미실행',
    'IDLE': '대기',
    'CHARGING': '도킹(충전 중)',
    'DOCKED': '도킹',
    'NAVIGATING': '이동 중(순찰 포함)',
    'DISPATCHING_EMERGENCY': '위급 출동 중',
    'DISPATCHING_HELMET': '안전모 확인 이동 중',
    'RETURNING_LOW_BATTERY': '배터리부족 복귀 중'
}

# 지도 표시용 상태 색상 (BGR)
STATE_COLOR = {
    'OFFLINE': (110, 110, 110),
    'NO_LOCALIZATION': (200, 100, 180),
    'NO_NAV2': (160, 60, 140),
    'IDLE': (230, 230, 230),
    'NAVIGATING': (200, 200, 80),
    'DISPATCHING_EMERGENCY': (60, 60, 255),
    'DISPATCHING_HELMET': (0, 165, 255),
    'RETURNING_LOW_BATTERY': (60, 220, 220),
    'CHARGING': (255, 190, 80),
    'DOCKED': (255, 190, 80),
}


class RobotContext:
    """ 관제가 추적하는 로봇 1대의 상태 """

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.fsm_state = 'OFFLINE'     # 관제 FSM 상태 (이 노드가 판단/배정한 상태)
        # 로봇의 실제 "구동 상태" 는 로봇의 자체 보고를 신뢰하지 않고
        # 각 로봇이 실제로 발행하는 표준 ROS 토픽에서 직접 판단한다:
        #   docked      <- /<robot>/dock_status (irobot_create_msgs/DockStatus.is_docked)
        #   navigating  <- /<robot>/navigate_to_pose/_action/status (Nav2 액션 상태)
        # reported_state 는 이 둘을 조합한 'DOCKED' | 'NAVIGATING' | 'IDLE' 중 하나
        self.docked = False
        self.navigating = False
        self.reported_state = None
        self.pose = None               # [x, y] (amcl_pose)
        self.battery = None            # 0.0 ~ 1.0
        self.last_seen = None          # 위 raw 토픽 중 하나라도 수신한 마지막 시각
        self.seen = {}                 # 토픽 소스별 마지막 수신 시각 (스택 세분 진단용)
        # 스택 세분 진단 결과 (tick 에서 갱신):
        #   None              정상
        #   NO_LOCALIZATION   battery_state 는 오는데 amcl_pose 미수신
        #                     -> localization 노드가 안 떠 있음
        #   NO_NAV2           battery+amcl 은 정상인데 planner_server/
        #                     smoother_server 노드가 그래프에 없음 -> nav2 미실행
        self.stack_fault = None
        self.dispatch_goal = None      # 진행 중 출동 좌표 [x, y]
        self.dispatch_reason = None    # 'EMERGENCY' | 'HELMET'
        self.dispatch_item = None      # 배정 원본 이벤트 (실패 시 재큐잉용)
        self.charging = False          # 배터리부족 복귀~충전완료 구간 (출동 배정 제외)
        # 출동 완료 보고가 없으므로(patrol_fire.py 는 이벤트를 보내지 않음),
        # Nav2 액션 상태의 navigating True->False 전이로 도착을 추론한다.
        # 명령 도달 전의 우연한 False(예: 이전 순찰 goal 잔상)를 도착으로
        # 오판하지 않으려면 '출동 후 한 번이라도 navigating=True 를
        # 관측했는지'를 먼저 확인해야 한다.
        self.dispatch_seen_moving = False
        # 로봇 구동 노드(amr_patrol_emer_helmet.py)가 amr_status 토픽으로
        # 보고하는 자체 상태 (IDLE/PATROL/EMERGENCY/HELMET/RETURNING).
        # 순찰/안전모 배정 시 "정말 idle 인지" 판단하는 근거로 쓴다.
        self.amr_status = None

    def update_reported_state(self):
        """ raw 토픽(docked/navigating) 조합으로 '구동 상태' 갱신 """
        if self.docked:
            self.reported_state = 'DOCKED'
        elif self.navigating:
            self.reported_state = 'NAVIGATING'
        else:
            self.reported_state = 'IDLE'

    def display_state(self):
        """ 표시용 상태 키: 관제 FSM(무엇을 하라고 시켰는지) 우선,
        그 외에는 raw 토픽 기반 구동 상태(DOCKED/NAVIGATING/IDLE) 표시 """
        if self.fsm_state == 'OFFLINE':
            return 'OFFLINE'
        if self.stack_fault:
            return self.stack_fault
        if self.fsm_state == 'DISPATCHING':
            return f'DISPATCHING_{self.dispatch_reason or "EMERGENCY"}'
        if self.fsm_state in ('RETURNING_LOW_BATTERY', 'CHARGING'):
            return self.fsm_state
        return self.reported_state or 'IDLE'
