# RAPID 스타일 프로그램 4개를 돌려 Isaac Sim 로봇에 ROS 2로 지령을 보내는 컨트롤러 노드
r"""실행: C:\isaacsim-6.2\python.bat rapid_controller.py

factory_line_demo.py를 그냥 실행하면 이 파일을 자식 프로세스로 같이 띄운다. 따로 띄워 보고 싶으면
터미널 1에서 factory_line_demo.py --no-controller, 터미널 2에서 이 파일을 실행한다.

이 노드는 Isaac Sim을 모른다. Isaac Sim의 함수를 하나도 부르지 않고, 아래 topic·service로만 로봇과 대화한다.
실제 공장이라면 로봇 컨트롤러 옆의 외부 PC에서 도는 프로그램 자리다. 로봇이 Isaac Sim이든 실제 로봇이든
같은 topic만 내보내면 이 파일은 고칠 필요가 없다.

    받는 것 (Isaac Sim → 이 노드)
      /clock                    rosgraph_msgs/Clock          시뮬레이션 시각. 받을 때마다 프로그램을 그만큼 진행한다
      /kuka/flange_pose         geometry_msgs/TransformStamped  KUKA 손끝 링크(tool0)의 실제 자세
      /kawasaki/flange_pose     geometry_msgs/TransformStamped  Kawasaki 손끝 링크(gripper_center)의 실제 자세
      /ammr/joint_states        sensor_msgs/JointState       AMMR 베이스 가상 관절 3개 + 팔 7축의 실제 값
      /h1/pose                  geometry_msgs/PoseStamped    H1 골반의 실제 자세
      /cell/di_box_at_pick      std_msgs/Bool                디지털 입력. 벨트 끝에 박스가 와 있다
      /cell/di_box_at_scan      std_msgs/Bool                디지털 입력. 박스가 검사 시작 위치를 지났다

    보내는 것 (이 노드 → Isaac Sim)
      /kuka/tcp_command         geometry_msgs/TransformStamped  지금 TCP가 있어야 할 자세. child_frame_id가 tool 이름
      /kawasaki/tcp_command     geometry_msgs/TransformStamped  위와 같다
      /ammr/joint_command       sensor_msgs/JointState       베이스 가상 관절 3개 + 팔 7축의 목표
      /h1/cmd_vel               geometry_msgs/TwistStamped   걷기 속도 명령 [전진 m/s, 횡 m/s, 회전 rad/s]
      /kawasaki/set_do_grip     std_srvs/SetBool (service)   SetDO doGrip. 그리퍼를 쥐고 편다

MoveL은 ROS 2 Action이 아니다. 경로 보간을 이 노드가 하고 0.005초마다 "지금 있어야 할 자세"를 topic으로 흘려보낸다.
ABB EGM이나 KUKA RSI처럼 외부 PC가 지령을 스트리밍하는 방식이다.
"""

import math

# line_common을 맨 먼저 불러온다. 그 안에서 RMW를 정하고 Isaac Sim에 들어 있는 rclpy 폴더를 찾을 수 있게 한다.
# 그래서 아래 rclpy·메시지 import보다 앞에 있어야 한다.
from line_common import (  # isort: skip
    AMMR_HOME,
    BELT_V,
    BELT_Z,
    BOX,
    H1_WAYPOINTS,
    PALLET_BUF,
    REACH,
    SCAN_LEN,
    SCAN_START,
    SLOT_X,
    TOOLS,
    TUCK,
    X_OUT,
    from_transform,
    seconds,
    stamp,
    to_transform,
)
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from rapid_style import (
    MoveAbsJ,
    MoveBase,
    MoveL,
    Offs,
    RapidTask,
    SetDO,
    WaitTime,
    WaitUntil,
    basetarget,
    fine,
    jointtarget,
    robtarget,
    v500,
    v1000,
    v1500,
    z50,
    z500,
)
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

# ------------------------------------------------------------------ RAPID 방식의 목표·공구 등록
# 이 노드는 Isaac Sim 장면을 볼 수 없다. 그래서 좌표는 숫자로 적는다. 물체 위의 점이 필요하면 Isaac Sim 창에서
# 그 물체를 클릭한다. 시뮬레이터 터미널에 "[TEACH] robtarget(...)" 한 줄이 찍히고, 그 줄을 여기에 붙인다.
# RobotStudio에서 스냅으로 찍은 타깃을 RAPID 모듈에 붙여 넣는 것과 같은 순서다.

# tooldata. 값은 line_common.py의 TOOLS에 있다. 시뮬레이터도 같은 값으로 IK를 한다.
tGripper = TOOLS["tGripper"]
tScanner = TOOLS["tScanner"]

# robtarget. [m] 위치 (x, y, z)와 [deg] 방향 (rx, ry, rz)이다. rx = 180은 공구가 바닥을 보는 자세다.
pPalletTop = robtarget(3.202, -1.499, 0.143, 180.0, 0.0, 0.0)  # [TEACH]로 찍은 /World/PalletOut 윗면 가운데
pPallet = Offs(pPalletTop, 0.0, 0.0, BOX)  # 팔레트 윗면에서 박스 한 개 높이만큼 올린 점. 박스를 놓을 때 TCP가 있을 곳
pPick = robtarget(X_OUT, 0.0, BELT_Z + BOX, 180.0, 0.0, 0.0)  # 벨트 끝에 온 박스의 윗면. 박스가 서는 위치는 라인에서 정한 값이다
pScanStart = robtarget(*SCAN_START, 180.0, 0.0, 0.0)  # 검사를 시작하는 점. 박스 윗면에서 10 cm 위
pScanEnd = Offs(pScanStart, SCAN_LEN, 0.0, 0.0)  # 검사를 끝내는 점. 벨트 방향으로 0.9 m 간 곳

# jointtarget. [deg] AMMR 팔 7축의 각도다. 시작 자세와 같은 값인 TUCK, REACH([rad])를 변환한다.
jTuck = jointtarget(*np.degrees(TUCK))  # (0, -57.3, 0, -137.5, 0, 91.7, 45.0) 접은 자세
jReach = jointtarget(*np.degrees(REACH))  # (0, 43.0, 0, -74.5, 0, 117.5, 45.0) 앞으로 뻗은 자세

# basetarget. [m] 몸체 위치 (x, y)와 [deg] 바라보는 방향 rz다 (0이 +x, 90이 +y).
bOut = basetarget(AMMR_HOME[0], AMMR_HOME[1], 90.0)  # 적재 팔레트 앞. 팔레트를 본다
bBuf = basetarget(PALLET_BUF[0], AMMR_HOME[1], 90.0)  # 출하 버퍼 팔레트 앞. 버퍼를 본다

# ------------------------------------------------------------------ 입출력 신호
# di는 Isaac Sim이 topic으로 보내 주는 디지털 입력의 마지막 값이다. 아래 노드가 받을 때마다 덮어쓴다.
# io는 출력 신호다. SetDO가 여기에 값을 쓰고, 노드가 값이 바뀐 것을 보고 Isaac Sim에 service를 부른다.
di = {"box_at_pick": False, "box_at_scan": False}
io = {"doGrip": 0}


def box_at_pick():
    """벨트 끝에 박스가 와 있으면 True. RAPID의 디지털 입력 diBoxAtPick에 해당한다."""
    return di["box_at_pick"]


def box_at_scan():
    """박스가 검사 시작 위치를 막 지났으면 True. RAPID의 디지털 입력 diBoxAtScan에 해당한다."""
    return di["box_at_scan"]


# ------------------------------------------------------------------ RAPID 프로그램 작성부
# 네 로봇의 동작은 아래 프로그램에서 목표(target)·속도(speed)·zone·tool과 명령 순서로 정의한다.
# 명령의 뜻과 ABB와 다른 점은 rapid_style.py 맨 위에 적혀 있다. 동작을 바꾸려면 이 구역의 *_program()을 수정한다.
#
# --- yield가 낯설다면
# 보통 함수는 return을 만나면 끝난다. yield가 들어 있는 함수는 다르다. yield를 만나면 그 값(명령 하나)을
# 내놓고 "그 자리에서 멈춰 기다린다". 다음에 다시 부르면 멈췄던 바로 다음 줄부터 이어서 실행한다.
# 실행기 RapidTask가 명령 하나를 받아 로봇이 그 동작을 끝낼 때까지 여러 스텝에 걸쳐 실행하고, 끝나면 다음 명령을 받는다.
# RAPID에서 MoveL 한 줄이 끝나야 프로그램 포인터(PP)가 다음 줄로 넘어가는 것과 같다.
# while True 안에 있으므로 마지막 줄까지 가면 처음으로 돌아가 끝없이 되풀이한다 (WHILE TRUE DO ... ENDWHILE).
def palletize_program():
    """Kawasaki 적재 프로그램. 같은 내용을 RAPID로 쓰면 아래와 같다 (단위만 mm에서 m로 다르다).

        PROC Palletize()
          WHILE TRUE DO
            pPlace := Offs(pPallet, slotX{nSlot}, 0, 0);
            MoveL Offs(pPick, 0, 0, 450), v1500, z50, tGripper;
            MoveL pPick, v1000, fine, tGripper;
            WaitUntil diBoxAtPick = 1;
            SetDO doGrip, 1;
            WaitTime 0.3;
            MoveL Offs(pPick, 0, 0, 450), v1000, z50, tGripper;
            MoveL Offs(pPlace, 0, 0, 600), v1500, z50, tGripper;
            MoveL pPlace, v1000, fine, tGripper;
            SetDO doGrip, 0;
            WaitTime 0.3;
            MoveL Offs(pPlace, 0, 0, 600), v1000, z50, tGripper;
            nSlot := (nSlot + 1) MOD 3;
          ENDWHILE
        ENDPROC

    yield 한 줄이 RAPID 명령 한 줄이다. 이동 시작 시각을 따로 적지 않는다. 걸리는 시간은 거리와 속도로 정해지고,
    박스가 오는 시각과는 WaitUntil로 맞춘다.
    fine에서는 Isaac Sim이 보내 준 실제 TCP 위치가 목표의 5 mm 안에 들어온 뒤에만 다음 줄로 넘어간다.
    SetDO는 io 딕셔너리를 바꾸고, 노드가 /kawasaki/set_do_grip service로 Isaac Sim에 알린다.
    '쥔다'는 시각용 박스 연결이며 RG2 손가락의 물리 접촉 파지는 아니다.
    """
    slot = 0  # 이번에 놓을 팔레트 칸 번호 (0, 1, 2)
    while True:
        pPlace = Offs(pPallet, SLOT_X[slot], 0.0, 0.0)  # 가운데 칸에서 x로 옮긴 이번 칸
        yield MoveL(Offs(pPick, 0.0, 0.0, 0.45), v1500, z50, tGripper)  # 집는 점 45 cm 위로 간다 (멈추지 않고 지나감)
        yield MoveL(pPick, v1000, fine, tGripper)  # 집는 점으로 내려가 정확히 선다
        yield WaitUntil(box_at_pick)  # 박스가 벨트 끝에 올 때까지 기다린다
        yield SetDO("doGrip", 1)  # 쥔다
        yield WaitTime(0.3)  # 쥐는 시간
        yield MoveL(Offs(pPick, 0.0, 0.0, 0.45), v1000, z50, tGripper)  # 들어 올린다
        yield MoveL(Offs(pPlace, 0.0, 0.0, 0.60), v1500, z50, tGripper)  # 놓을 칸 60 cm 위로 옮긴다
        yield MoveL(pPlace, v1000, fine, tGripper)  # 놓을 칸으로 내려가 정확히 선다
        yield SetDO("doGrip", 0)  # 놓는다
        yield WaitTime(0.3)  # 놓는 시간
        yield MoveL(Offs(pPlace, 0.0, 0.0, 0.60), v1000, z50, tGripper)  # 빈손으로 올린다
        slot = (slot + 1) % 3  # 다음 칸


def scan_program():
    """KUKA 검사 프로그램. 지나가는 박스 위 10 cm에서 박스와 나란히 이동한 뒤 돌아온다.

        PROC Scan()
          WHILE TRUE DO
            MoveL Offs(pScanStart, 0, 0, 400), v1000, z50, tScanner;
            MoveL pScanStart, v1000, fine, tScanner;
            WaitUntil diBoxAtScan = 1;
            MoveL pScanEnd, vBelt, fine, tScanner;
            MoveL Offs(pScanEnd, 0, 0, 400), v1000, z50, tScanner;
          ENDWHILE
        ENDPROC

    컨베이어 트래킹은 아니다. 박스가 시작 위치에 온 것을 신호로 받아 벨트와 같은 속도(vBelt)로 직선 이동할 뿐이다.
    """
    vBelt = BELT_V  # [m/s] 벨트와 같은 속도. speeddata를 직접 만든 것이다
    while True:
        yield MoveL(Offs(pScanStart, 0.0, 0.0, 0.40), v1000, z50, tScanner)  # 시작 위치 40 cm 위로 돌아온다
        yield MoveL(pScanStart, v1000, fine, tScanner)  # 박스가 지나갈 높이로 내려가 정확히 선다
        yield WaitUntil(box_at_scan)  # 박스가 바로 아래에 올 때까지 기다린다
        yield MoveL(pScanEnd, vBelt, fine, tScanner)  # 박스와 같은 속도로 나란히 간다
        yield MoveL(Offs(pScanEnd, 0.0, 0.0, 0.40), v1000, z50, tScanner)  # 올라간다


def ammr_program():
    """AMMR 프로그램. 적재 팔레트와 출하 버퍼 사이를 오가며 도착할 때마다 팔을 뻗었다 접는다.

    MoveBase가 몸체(바퀴 베이스)를, MoveAbsJ가 그 위의 팔 7축을 움직인다. 모두 fine이라 실제로 도착한 뒤에
    다음 줄로 넘어간다. MoveBase는 RAPID에 없는 명령이지만 모양은 MoveL과 같다 (목표, 속도, zone).
    제자리 회전은 x, y가 같고 rz만 다른 목표를 주면 된다.
    """
    while True:
        yield MoveBase(bOut, v500, fine)  # 적재 팔레트 앞에서 팔레트 쪽(+y)을 보고 선다
        yield MoveAbsJ(jReach, 45.0, fine)  # 팔을 앞으로 뻗는다. 45는 [deg/s]
        yield WaitTime(1.0)
        yield MoveAbsJ(jTuck, 45.0, fine)  # 팔을 접는다
        yield MoveBase(basetarget(AMMR_HOME[0], AMMR_HOME[1], 180.0), v500, fine)  # 제자리에서 갈 방향(-x)으로 돈다
        yield MoveBase(basetarget(PALLET_BUF[0], AMMR_HOME[1], 180.0), v500, fine)  # 버퍼 앞까지 직진한다
        yield MoveBase(bBuf, v500, fine)  # 버퍼 팔레트 쪽(+y)을 보고 선다
        yield MoveAbsJ(jReach, 45.0, fine)
        yield WaitTime(1.0)
        yield MoveAbsJ(jTuck, 45.0, fine)
        yield MoveBase(basetarget(PALLET_BUF[0], AMMR_HOME[1], 0.0), v500, fine)  # 제자리에서 갈 방향(+x)으로 돈다
        yield MoveBase(basetarget(AMMR_HOME[0], AMMR_HOME[1], 0.0), v500, fine)  # 적재 팔레트 앞까지 직진한다


def patrol_program():
    """H1 순찰 프로그램. 순찰 지점 네 곳을 차례로 걸어서 돈다.

    H1은 걷는 로봇이라 MoveBase의 뜻이 조금 다르다. 목표 쪽으로 몸을 돌려 걸어가고, 도착 방향 rz는 맞추지 않는다.
    z500이라 지점 0.5 m 안에 들어오면 멈추지 않고 다음 지점으로 향한다. 속도는 정책이 학습된 0 ~ 1 m/s 안에서 준다.
    """
    while True:
        for x, y in H1_WAYPOINTS:
            yield MoveBase(basetarget(x, y, 0.0), v500, z500)


def walk_command(pose, goal, speed=0.5):
    """H1을 [m] goal (x, y)로 보내는 [전진 m/s, 횡 m/s, 회전 rad/s] 명령. goal이 None이면 제자리에 선다.

    pose는 H1의 지금 실제 자세 ([m] x, [m] y, [rad] 방향)다.
    목표 방향과 몸 방향의 차이가 0.6 rad 미만이면 [m/s] speed로 걷고, 크면 제자리에서 돈다.
    ROS 2의 이동 로봇은 보통 이렇게 /cmd_vel 속도 명령으로 움직인다. 걷는 방법은 Isaac Sim 쪽 보행 정책이 정한다.
    """
    if goal is None:
        return 0.0, 0.0, 0.0
    x, y, yaw = pose
    turn = (math.atan2(goal[1] - y, goal[0] - x) - yaw + math.pi) % (2 * math.pi) - math.pi
    return (speed if abs(turn) < 0.6 else 0.0), 0.0, float(np.clip(1.5 * turn, -0.7, 0.7))


def yaw_of(q):
    """쿼터니언 메시지(x, y, z, w)에서 [rad] 위에서 본 방향(z축 회전)을 꺼낸다."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# ------------------------------------------------------------------ ROS 2 노드
class RapidController(Node):
    """RAPID 태스크 4개를 시뮬레이션 시계에 맞춰 돌리는 ROS 2 노드. RobotStudio의 컨트롤러 하나에 해당한다.

    __init__에서 topic 구독·발행과 service 클라이언트를 만든다. 그 뒤로는 /clock 메시지가 올 때마다
    on_clock()이 한 번 불린다. 나머지 구독 콜백은 "마지막으로 받은 값"을 저장만 한다.
    """

    def __init__(self):
        super().__init__("rapid_controller")
        self.latest = {}  # 마지막으로 받은 로봇 상태. 키는 topic 이름이다
        self.tasks = None  # 로봇마다의 RapidTask. 첫 /clock을 받을 때와 시계가 되돌아갈 때 새로 만든다
        self.clock = None  # [s] 마지막으로 처리한 시뮬레이션 시각
        self.sent_grip = None  # Isaac Sim에 마지막으로 보낸 doGrip 값
        # 구독. create_subscription(메시지 타입, topic 이름, 받았을 때 부를 함수, 큐 길이)이다.
        for topic, kind in (
            ("/kuka/flange_pose", TransformStamped),
            ("/kawasaki/flange_pose", TransformStamped),
            ("/ammr/joint_states", JointState),
            ("/h1/pose", PoseStamped),
        ):
            self.create_subscription(kind, topic, lambda msg, topic=topic: self.latest.__setitem__(topic, msg), 10)
        self.create_subscription(Bool, "/cell/di_box_at_pick", lambda msg: di.__setitem__("box_at_pick", msg.data), 10)
        self.create_subscription(Bool, "/cell/di_box_at_scan", lambda msg: di.__setitem__("box_at_scan", msg.data), 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        # 발행. create_publisher(메시지 타입, topic 이름, 큐 길이)이다.
        self.pub = {
            "kuka": self.create_publisher(TransformStamped, "/kuka/tcp_command", 10),
            "kawasaki": self.create_publisher(TransformStamped, "/kawasaki/tcp_command", 10),
            "ammr": self.create_publisher(JointState, "/ammr/joint_command", 10),
            "h1": self.create_publisher(TwistStamped, "/h1/cmd_vel", 10),
        }
        # service 클라이언트. SetDO doGrip을 Isaac Sim에 요청한다. 요청 하나에 응답 하나로 끝나는 통신이다.
        self.grip = self.create_client(SetBool, "/kawasaki/set_do_grip")
        self.get_logger().info("waiting for /clock from Isaac Sim")

    def restart(self):
        """프로그램을 첫 줄부터 다시 시작한다. RobotStudio에서 PP to Main을 누르는 것과 같다."""
        io["doGrip"] = 0
        self.sent_grip = None
        self.tasks = {
            "kuka": RapidTask(scan_program(), io),
            "kawasaki": RapidTask(palletize_program(), io),
            "ammr": RapidTask(ammr_program(), io),
            "h1": RapidTask(patrol_program(), io, walk=True),
        }

    def on_clock(self, msg):
        """시뮬레이션 시각이 올 때마다 네 프로그램을 그 시간차만큼 진행하고 지령을 보낸다.

        로봇 상태는 구독 콜백이 저장해 둔 마지막 값을 쓴다. 아직 하나라도 못 받았으면 이번 시각은 건너뛴다.
        """
        now = seconds(msg.clock)  # [s] 시뮬레이션이 재생을 시작한 뒤 흐른 시간
        if self.clock is not None and now < self.clock:  # 시계가 되돌아갔다. Isaac Sim에서 정지 후 다시 재생한 것이다
            self.get_logger().info("simulation restarted, programs back to the first line")
            self.tasks = None
        if any(topic not in self.latest for topic in ("/kuka/flange_pose", "/kawasaki/flange_pose", "/ammr/joint_states", "/h1/pose")):
            self.clock = now
            return
        if self.tasks is None:
            self.restart()
            self.clock = now
            return
        dt, self.clock = now - self.clock, now  # [s] 지난번에 처리한 뒤 흐른 시뮬레이션 시간 (보통 물리 한 스텝 0.005)
        if dt <= 0.0:
            return
        tasks = self.tasks
        # 팔 두 대. 손끝 링크의 실제 자세를 넘기면 RapidTask가 MoveL을 진행해 "지금 TCP가 있어야 할 자세"를 만든다.
        for name in ("kuka", "kawasaki"):
            tasks[name].step(dt, tcp=from_transform(self.latest[f"/{name}/flange_pose"]))
            if tasks[name].pos is not None:  # 첫 MoveL 전에는 지령이 없다. Isaac Sim이 시작 자세를 유지한다
                tool_name = next((k for k, v in TOOLS.items() if v is tasks[name].tool), None)
                if tool_name is None:  # 지령에는 이름만 실려 가므로 시뮬레이터가 같은 tooldata를 찾을 수 있어야 한다
                    raise ValueError(f"{name}: MoveL의 tool이 line_common.TOOLS에 없다. 새 tooldata는 TOOLS에 이름과 함께 추가한다")
                self.pub[name].publish(to_transform(now, tasks[name].pos, tasks[name].rot, tool_name))
        # AMMR. 베이스 가상 관절값은 처음 놓은 자리(AMMR_HOME)가 0이다. RapidTask는 월드 좌표로 계산하므로 더하고 뺀다.
        q = np.array(self.latest["/ammr/joint_states"].position)
        tasks["ammr"].step(dt, base=(q[0] + AMMR_HOME[0], q[1] + AMMR_HOME[1], q[2]), joints=q[3:10])
        base, arm_joints = tasks["ammr"].base, tasks["ammr"].joints
        command = JointState()
        command.header.stamp = stamp(now)
        command.name = list(self.latest["/ammr/joint_states"].name)
        command.position = [base[0] - AMMR_HOME[0], base[1] - AMMR_HOME[1], float(base[2]), *map(float, arm_joints)]
        self.pub["ammr"].publish(command)
        # H1. 프로그램은 "어디로 얼마나 빨리"만 정한다. 그것을 속도 명령으로 바꿔 보낸다. 걷는 것은 Isaac Sim의 보행 정책이다.
        h1 = self.latest["/h1/pose"].pose
        pose = (h1.position.x, h1.position.y, yaw_of(h1.orientation))
        tasks["h1"].step(dt, base=pose)
        goal = tasks["h1"].goal
        forward, side, turn = walk_command(pose, None) if goal is None else walk_command(pose, goal[0], goal[1])
        twist = TwistStamped()
        twist.header.stamp = stamp(now)
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z = forward, side, turn
        self.pub["h1"].publish(twist)
        # 출력 신호. SetDO가 io 값을 바꿨으면 service를 부른다. 응답은 기다리지 않는다 (call_async).
        # 기다리면 이 함수가 멈추고, 이 함수가 멈추면 /clock 처리도 멈춘다.
        if io["doGrip"] != self.sent_grip and self.grip.service_is_ready():
            self.grip.call_async(SetBool.Request(data=bool(io["doGrip"])))
            self.sent_grip = io["doGrip"]


def main():
    rclpy.init()  # ROS 2 통신을 시작한다. 같은 네트워크(같은 ROS_DOMAIN_ID)의 다른 노드를 찾기 시작한다
    node = RapidController()
    try:
        rclpy.spin(node)  # 메시지가 올 때마다 위의 콜백을 부른다. Ctrl+C나 종료 신호가 올 때까지 돈다
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
