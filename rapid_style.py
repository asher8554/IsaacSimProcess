# ABB RAPID 방식(robtarget·speeddata·zonedata·tooldata·MoveL)으로 쓴 프로그램을 스텝마다의 목표로 바꿔 주는 층
r"""RobotStudio에서 쓰던 방식 그대로 "좌표 + 속도 + zone + tool"로 동작을 적을 수 있게 해 준다.

산업용 로봇의 팔(MoveL)뿐 아니라 이동 로봇의 베이스(MoveBase), 관절 직접 이동(MoveAbsJ)도 같은 모양으로 적는다.
Isaac Sim과 무관한 순수 계산이다. 그래서 이 파일만 따로 실행해 시험할 수 있다.

    C:\isaacsim-6.2\python.bat rapid_style.py

RAPID와 이 파일의 대응.

    RAPID                                   이 파일
    ------------------------------------    ----------------------------------------
    CONST robtarget p := [[x,y,z],[q],..];  p = robtarget(x, y, z, rx, ry, rz)
    CONST jointtarget j := [[j1..j6],..];   j = jointtarget(j1, j2, ...)
    (없음)                                  b = basetarget(x, y, rz)
    Offs(p, dx, dy, dz)                     Offs(p, dx, dy, dz)
    v1000, z50, fine                        v1000, z50, fine
    PERS tooldata t := [...];               t = tooldata(x, y, z, rx, ry, rz)
    MoveL p, v1000, z50, t;                 yield MoveL(p, v1000, z50, t)
    MoveAbsJ j, v1000, fine, t;             yield MoveAbsJ(j, 45.0, fine)
    (없음)                                  yield MoveBase(b, v500, fine)
    WaitTime 0.3;                           yield WaitTime(0.3)
    WaitUntil di1 = 1;                      yield WaitUntil(조건_함수)
    SetDO do1, 1;                           yield SetDO("do1", 1)
    WHILE TRUE DO ... ENDWHILE              while True: ...

ABB와 다른 점.

- 길이는 mm가 아니라 m, 속도는 mm/s가 아니라 m/s다. v1000은 1.0 m/s, z50은 0.05 m다.
- 방향은 쿼터니언 대신 오일러 각 (rx, ry, rz) [deg]로 적는다. 회전 순서는 RAPID의 OrientZYX와 같다.
  rx = 180, ry = 0, rz = 0이면 공구 Z축이 아래를 본다 (ABB 쿼터니언 [0, 1, 0, 0]과 같은 자세).
- MoveJ, wobj, 외부 축은 없다. 좌표는 모두 월드 기준이다.
- zone은 모서리를 포물선으로 깎지 않는다. 목표의 zone 반경 안에 들어오면 멈추지 않고 다음 목표로 방향을 튼다.
- MoveAbsJ의 속도는 TCP 속도가 아니라 [deg/s] 가장 많이 움직이는 관절의 속도다.
- MoveBase는 RAPID에 없는 명령이다. 이동 로봇의 몸체를 바닥 위 (x, y)와 방향 rz로 보낸다.
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

ACCEL = 4.0  # [m/s²] TCP가 속도를 올리고 내리는 한계. RAPID의 AccSet에 해당한다
BASE_ACCEL = 1.0  # [m/s²] 이동 로봇 베이스가 속도를 올리고 내리는 한계
JOINT_ACCEL = math.radians(360.0)  # [rad/s²] MoveAbsJ에서 관절이 속도를 올리고 내리는 한계
TURN_RADIUS = 0.5  # [m] 베이스 회전을 이동 거리로 환산하는 반경. 90도 회전을 0.79 m 이동과 같은 시간에 한다
IN_POSITION = 0.005  # [m] 정지점(fine)에서 실제 TCP가 목표에 이만큼 가까워져야 도착으로 본다
BASE_IN_POSITION = 0.03  # [m] 정지점에서 실제 베이스가 목표에 이만큼 가까워져야 도착으로 본다
JOINT_IN_POSITION = math.radians(1.0)  # [rad] 정지점에서 실제 관절이 목표에 이만큼 가까워져야 도착으로 본다
WALK_ARRIVE = 0.2  # [m] 걷는 로봇은 정지점이라도 이 거리 안에 들어오면 도착으로 본다

# speeddata. [m/s] TCP나 베이스의 속도. 이름의 숫자는 ABB와 같이 mm/s다.
v100, v200, v500, v1000, v1500, v2000 = 0.1, 0.2, 0.5, 1.0, 1.5, 2.0
# zonedata. [m] 목표에서 이 거리 안에 들어오면 다음 명령으로 넘어간다. fine은 목표에 정확히 서는 정지점이다.
fine, z1, z10, z50, z100, z200, z500 = 0.0, 0.001, 0.01, 0.05, 0.1, 0.2, 0.5


# ------------------------------------------------------------------ 데이터 (robtarget, tooldata, jointtarget, basetarget)
def robtarget(x, y, z, rx, ry, rz):
    """목표 자세 하나. (x, y, z)는 [m] TCP가 가야 할 월드 좌표, (rx, ry, rz)는 [deg] TCP의 방향이다.

    방향은 월드 축을 z로 rz, 그다음 y로 ry, 그다음 x로 rx만큼 차례로 돌린 것이다 (RAPID OrientZYX(rz, ry, rx)).
    """
    return {"pos": np.array([x, y, z], dtype=float), "rot": Rotation.from_euler("ZYX", [rz, ry, rx], degrees=True)}


def tooldata(x, y, z, rx, ry, rz):
    """공구 정의. 로봇 손끝 링크에서 본 TCP의 위치 [m]와 방향 [deg]이다 (RAPID tooldata의 tframe).

    TCP의 Z축이 공구가 뻗어 나가는 방향이 되도록 방향을 정한다. ABB tool0과 같은 약속이다.
    """
    return robtarget(x, y, z, rx, ry, rz)


def jointtarget(*degrees):
    """관절 목표. 관절마다의 각도를 [deg]로 차례로 적는다 (RAPID jointtarget의 robax)."""
    return np.radians(np.array(degrees, dtype=float))


def basetarget(x, y, rz):
    """이동 로봇 몸체의 목표. (x, y)는 [m] 바닥 위 월드 좌표, rz는 [deg] 몸이 바라보는 방향이다 (0이 +x, 90이 +y)."""
    return {"pos": np.array([x, y], dtype=float), "yaw": math.radians(rz)}


def Offs(target, dx, dy, dz):
    """robtarget에서 월드 x·y·z 방향으로 [m] (dx, dy, dz)만큼 옮긴 새 목표를 돌려준다. 방향은 그대로다."""
    return {"pos": target["pos"] + np.array([dx, dy, dz], dtype=float), "rot": target["rot"]}


# ------------------------------------------------------------------ 명령
# 프로그램은 이 명령들을 yield로 하나씩 내놓는다. 여기서는 내용을 묶어 두기만 하고 실행은 RapidTask가 한다.
def MoveL(target, speed, zone, tool):
    """TCP를 robtarget까지 직선으로 보낸다. speed는 [m/s], zone은 [m]이다."""
    return ("MoveL", target, speed, zone, tool)


def MoveAbsJ(target, speed, zone):
    """관절을 jointtarget까지 한꺼번에 보낸다. speed는 [deg/s] 가장 많이 움직이는 관절의 속도다.

    zone은 fine이냐 아니냐만 본다. fine이면 실제 관절이 도착할 때까지 기다린다.
    """
    return ("MoveAbsJ", target, math.radians(speed), zone)


def MoveBase(target, speed, zone):
    """이동 로봇의 몸체를 basetarget까지 보낸다. speed는 [m/s], zone은 [m]이다.

    바퀴 로봇(drive)은 직선으로 가면서 방향 rz도 같이 맞춘다. 제자리 회전은 x, y가 같은 목표를 주면 된다.
    걷는 로봇(walk)은 목표 쪽으로 몸을 돌려 걸어가고 rz는 맞추지 않는다.
    """
    return ("MoveBase", target, speed, zone)


def WaitTime(seconds):
    """[s] 지정한 시간 동안 그 자리에서 기다린다."""
    return ("WaitTime", seconds)


def WaitUntil(condition):
    """condition()이 참이 될 때까지 그 자리에서 기다린다. 센서 입력을 기다릴 때 쓴다."""
    return ("WaitUntil", condition)


def SetDO(name, value):
    """출력 신호 name을 value로 바꾼다. 그리퍼를 쥐고 펴는 신호에 쓴다."""
    return ("SetDO", name, value)


# ------------------------------------------------------------------ 실행기
class RapidTask:
    """프로그램 하나를 물리 스텝마다 조금씩 실행한다. RobotStudio의 태스크 하나에 해당한다.

    program  명령을 yield하는 제너레이터 (아래 시험 코드나 factory_line_demo.py의 프로그램 참고).
    io       출력 신호를 담는 dict. SetDO가 여기에 값을 쓴다.
    walk     True면 MoveBase를 "걸어가는 로봇"으로 다룬다. 지령 경로를 만들지 않고 목표만 알려 준다.

    step()을 부르고 나면 아래 값에 "지금 로봇이 있어야 할 상태"가 들어 있다. 그 상태로 로봇을 실제로
    보내는 일은 부르는 쪽이 한다.

        pos, rot, tool   MoveL이 만든 지령 TCP 위치 [m]·방향과 그때 쓴 tooldata. MoveL 전에는 None이다.
        joints           MoveAbsJ가 만든 지령 관절각 [rad].
        base             MoveBase(drive)가 만든 지령 몸체 자세 [m, m, rad] (x, y, 방향).
        goal             MoveBase(walk)의 목표 ([m] (x, y), [m/s] 속도). 걸을 일이 없으면 None이다.
    """

    def __init__(self, program, io, walk=False):
        self.program = program
        self.io = io
        self.walk = walk
        self.command = None  # 지금 실행 중인 명령. None이면 다음 명령을 꺼낸다
        self.kind = None  # 직전 명령의 종류. 같은 종류가 이어질 때만 속도를 이어받는다
        self.pos, self.rot, self.tool = None, None, None
        self.joints = None
        self.base = None
        self.goal = None
        self.speed = 0.0  # 지금 지령 속도. MoveL·MoveBase는 [m/s], MoveAbsJ는 [rad/s]
        self.waited = 0.0  # [s] WaitTime에서 지금까지 기다린 시간

    def step(self, dt, tcp=None, joints=None, base=None):
        """[s] dt만큼 프로그램을 진행한다. 인자는 로봇의 지금 실제 상태이고 쓰는 것만 넘기면 된다.

        tcp     (손끝 링크의 [m] 위치, 방향). MoveL을 쓰는 로봇이 넘긴다.
        joints  [rad] 관절각. MoveAbsJ를 쓰는 로봇이 넘긴다.
        base    ([m] x, [m] y, [rad] 방향). MoveBase를 쓰는 로봇이 넘긴다.

        실제 상태는 두 곳에 쓴다. 프로그램을 시작할 때 "지금 여기서 출발한다"고 정하는 데와,
        정지점(fine)에서 로봇이 실제로 도착했는지 확인하는 데다.
        """
        if self.joints is None and joints is not None:
            self.joints = np.array(joints, dtype=float)
        if self.base is None and base is not None:
            self.base = np.array(base, dtype=float)
        for _ in range(20):  # 시간이 안 드는 명령(SetDO, 이미 참인 WaitUntil)은 같은 스텝에 이어서 처리한다
            if self.command is None:
                self.command = next(self.program)
                self.waited = 0.0
                if self.command[0] != self.kind:  # 종류가 바뀌면 속도 단위도 다르므로 0에서 다시 시작한다
                    self.speed = 0.0
                self.kind = self.command[0]
            kind = self.command[0]
            if kind == "SetDO":
                self.io[self.command[1]] = self.command[2]
                self.command = None
            elif kind == "WaitUntil":
                if not self.command[1]():
                    self.speed = 0.0
                    break
                self.command = None
            elif kind == "WaitTime":
                self.speed = 0.0
                self.waited += dt
                if self.waited >= self.command[1]:
                    self.command = None
                break
            else:  # 움직이는 명령은 한 스텝에 하나만 진행한다
                moved = {"MoveL": self._move_l, "MoveAbsJ": self._move_abs_j, "MoveBase": self._move_base}[kind]
                if moved(dt, tcp, joints, base):
                    self.command = None
                break

    def _profile(self, remaining, speed, accel, stop, dt):
        """남은 거리 remaining 중 이번 스텝에 갈 비율(0~1)을 돌려준다. 1이면 이번 스텝에 목표에 닿는다.

        가속 한계 안에서 speed까지 올리고, stop이면 목표에 가까워질수록 줄여서 정확히 선다.
        거리와 속도의 단위는 [m]와 [m/s]이거나 [rad]와 [rad/s]다.
        """
        velocity = min(speed, self.speed + accel * dt)
        if stop:
            velocity = min(velocity, math.sqrt(2.0 * accel * remaining))
        advance = velocity * dt
        if remaining <= advance:
            self.speed = 0.0 if stop else velocity
            return 1.0
        self.speed = velocity
        return advance / remaining

    def _move_l(self, dt, tcp, joints, base):
        """MoveL 한 스텝. 지령 TCP를 목표 쪽으로 직선으로 조금 옮긴다. 명령이 끝났으면 True."""
        _, target, speed, zone, tool = self.command
        link_pos, link_rot = tcp
        if self.pos is None or tool is not self.tool:  # 시작할 때와 tool이 바뀔 때 현재 TCP 자세에서 출발한다
            self.tool = tool
            self.pos = link_pos + link_rot.apply(tool["pos"])
            self.rot = link_rot * tool["rot"]
        to_go = target["pos"] - self.pos
        distance = float(np.linalg.norm(to_go))  # [m] 목표까지 남은 거리
        fraction = self._profile(distance, speed, ACCEL, zone == fine, dt)
        if fraction < 1.0:
            # 위치는 직선으로, 방향은 남은 거리에 비례해서 같이 돌린다. 목표에 닿을 때 방향도 맞는다.
            self.pos = self.pos + to_go * fraction
            self.rot = Rotation.from_rotvec((target["rot"] * self.rot.inv()).as_rotvec() * fraction) * self.rot
            # zone이 있으면 반경 안에 들어온 순간 끝낸다. 멈추지 않고 다음 명령이 이어받는다.
            return zone > 0.0 and distance * (1.0 - fraction) <= zone
        self.pos, self.rot = target["pos"].copy(), target["rot"]
        if zone != fine:
            return True
        # 정지점이면 실제 TCP가 도착할 때까지 기다린다. 지령이 닿아도 실제 로봇은 조금 늦게 온다.
        actual = link_pos + link_rot.apply(tool["pos"])
        return float(np.linalg.norm(actual - target["pos"])) <= IN_POSITION

    def _move_abs_j(self, dt, tcp, joints, base):
        """MoveAbsJ 한 스텝. 모든 관절을 같은 비율로 목표 쪽으로 조금 옮긴다. 명령이 끝났으면 True."""
        _, target, speed, zone = self.command
        to_go = target - self.joints
        fraction = self._profile(float(np.max(np.abs(to_go))), speed, JOINT_ACCEL, True, dt)
        self.joints = self.joints + to_go * fraction
        if fraction < 1.0:
            return False
        return zone != fine or float(np.max(np.abs(np.asarray(joints) - target))) <= JOINT_IN_POSITION

    def _move_base(self, dt, tcp, joints, base):
        """MoveBase 한 스텝. 명령이 끝났으면 True."""
        _, target, speed, zone = self.command
        actual = np.asarray(base, dtype=float)
        if self.walk:
            # 걷는 로봇. 지령 경로를 만들지 않는다. 목표만 내놓고 실제로 도착했는지만 본다.
            if float(np.linalg.norm(actual[:2] - target["pos"])) <= max(zone, WALK_ARRIVE):
                self.goal = None
                return True
            self.goal = (target["pos"], speed)
            return False
        # 바퀴 로봇. 지령 몸체 자세를 목표 쪽으로 직선으로 조금 옮기고 방향도 같은 비율로 돌린다.
        to_go = target["pos"] - self.base[:2]
        turn = (target["yaw"] - self.base[2] + math.pi) % (2.0 * math.pi) - math.pi  # [rad] 가까운 쪽으로 도는 각도
        remaining = max(float(np.linalg.norm(to_go)), abs(turn) * TURN_RADIUS)
        fraction = self._profile(remaining, speed, BASE_ACCEL, zone == fine, dt)
        self.base = self.base + np.array([to_go[0], to_go[1], turn]) * fraction
        if fraction < 1.0:
            return zone > 0.0 and remaining * (1.0 - fraction) <= zone
        if zone != fine:
            return True
        off_heading = abs((actual[2] - target["yaw"] + math.pi) % (2.0 * math.pi) - math.pi)
        return float(np.linalg.norm(actual[:2] - target["pos"])) <= BASE_IN_POSITION and off_heading <= math.radians(2.0)


# ------------------------------------------------------------------ 단독 시험
if __name__ == "__main__":
    DT = 0.005  # [s] 시험용 시간 간격 (200 Hz)
    still = Rotation.identity()
    origin = np.zeros(3)
    tool0 = tooldata(0, 0, 0, 0, 0, 0)

    def run(program, seconds, io=None):
        """프로그램을 seconds 동안 돌리고 스텝마다의 (시각, 위치, 속도)와 마지막 방향을 돌려준다.

        로봇은 지령을 한 스텝 늦게 그대로 따라온다고 가정한다 (tool0이라 손끝 링크 = TCP).
        """
        task = RapidTask(program, {} if io is None else io)
        log, link_pos, link_rot = [], origin, still
        for i in range(int(seconds / DT)):
            task.step(DT, tcp=(link_pos, link_rot))
            log.append((i * DT, task.pos.copy(), task.speed))
            link_pos, link_rot = task.pos.copy(), task.rot
        return log, task.rot

    # 1. 정지점. 1 m를 v500으로 가면 정확히 목표에 서고, 걸린 시간은 거리/속도 + 가감속 시간과 맞는다.
    def program_fine():
        yield MoveL(robtarget(1, 0, 0, 180, 0, 90), v500, fine, tool0)
        yield WaitTime(10.0)

    log, rot = run(program_fine(), 4.0)
    arrived = next(t for t, pos, _ in log if np.allclose(pos, [1, 0, 0]))
    expected = 1.0 / v500 + v500 / ACCEL
    assert abs(arrived - expected) < 0.05, (arrived, expected)
    assert log[-1][2] == 0.0
    assert np.allclose(rot.as_matrix(), Rotation.from_euler("ZYX", [90, 0, 180], degrees=True).as_matrix(), atol=1e-9)
    print(f"PASS fine point: arrived at {arrived:.3f} s (expected {expected:.3f} s), stopped, orientation reached")

    # 2. zone. 모서리 B를 z50으로 지나면 B에 닿지 않고 5 cm 안쪽으로 돌아가며, 그 구간에서 속도가 줄지 않는다.
    def program_zone():
        yield MoveL(robtarget(1, 0, 0, 180, 0, 0), v500, z50, tool0)
        yield MoveL(robtarget(1, 1, 0, 180, 0, 0), v500, fine, tool0)
        yield WaitTime(10.0)

    log, _ = run(program_zone(), 6.0)
    closest = min(float(np.linalg.norm(pos - [1, 0, 0])) for _, pos, _ in log)
    slowest = min(speed for t, _, speed in log if 0.5 < t < 3.0)
    assert 0.03 < closest <= z50, closest
    assert slowest >= v500 - 1e-9, slowest
    assert np.allclose(log[-1][1], [1, 1, 0])
    print(f"PASS zone: passed corner at {closest:.3f} m without stopping (min speed {slowest:.2f} m/s), reached end")

    # 3. 신호와 대기. WaitUntil이 참이 된 뒤에만 다음으로 넘어가고, SetDO가 순서대로 실행된다.
    signals, gate = {"grip": 0}, {"open": False}

    def program_io():
        yield WaitUntil(lambda: gate["open"])
        yield SetDO("grip", 1)
        yield WaitTime(0.2)
        yield SetDO("grip", 0)
        yield WaitTime(10.0)

    task = RapidTask(program_io(), signals)
    for _ in range(100):
        task.step(DT)
    assert signals["grip"] == 0  # 조건이 거짓인 동안은 넘어가지 않는다
    gate["open"] = True
    task.step(DT)
    assert signals["grip"] == 1
    for _ in range(int(0.2 / DT) + 2):
        task.step(DT)
    assert signals["grip"] == 0
    print("PASS io: WaitUntil held the program, SetDO ran in order around WaitTime")

    # 4. 정지점은 실제 로봇이 도착할 때까지 기다린다. 로봇이 안 따라오면 다음 명령(SetDO)으로 넘어가지 않는다.
    signals = {"grip": 0}
    down = Rotation.from_euler("ZYX", [0, 0, 180], degrees=True)

    def program_wait_robot():
        yield MoveL(robtarget(0.1, 0, 0, 180, 0, 0), v500, fine, tool0)
        yield SetDO("grip", 1)
        yield WaitTime(10.0)

    task = RapidTask(program_wait_robot(), signals)
    for _ in range(400):  # 2초. 지령은 목표에 닿지만 로봇(손끝 링크)은 원점에 그대로 있다
        task.step(DT, tcp=(origin, down))
    assert signals["grip"] == 0
    for _ in range(2):  # 로봇이 도착했다. 첫 스텝에 MoveL이 끝나고 다음 스텝에 SetDO가 실행된다
        task.step(DT, tcp=(np.array([0.1, 0.0, 0.0]), down))
    assert signals["grip"] == 1
    print("PASS fine waits for the robot: SetDO ran only after the actual TCP reached the target")

    # 5. MoveAbsJ. 가장 많이 움직이는 관절이 지정 속도로 가고, 모든 관절이 같은 순간에 도착한다.
    def program_joints():
        yield MoveAbsJ(jointtarget(90.0, -45.0, 0.0), 45.0, fine)
        yield WaitTime(10.0)

    task = RapidTask(program_joints(), {})
    actual, arrived, halfway = np.zeros(3), None, None
    for i in range(int(4.0 / DT)):
        task.step(DT, joints=actual)
        actual = task.joints.copy()  # 로봇이 지령을 그대로 따라온다
        if halfway is None and abs(math.degrees(actual[0]) - 45.0) < 0.2:
            halfway = np.degrees(actual)
        if arrived is None and np.allclose(np.degrees(actual), [90.0, -45.0, 0.0]):
            arrived = i * DT
    expected = 90.0 / 45.0 + math.radians(45.0) / JOINT_ACCEL
    assert arrived is not None and abs(arrived - expected) < 0.05, (arrived, expected)
    assert abs(halfway[1] + 22.5) < 0.2, halfway  # 첫 관절이 절반 갔을 때 둘째 관절도 절반 갔다
    print(f"PASS MoveAbsJ: arrived at {arrived:.3f} s (expected {expected:.3f} s), joints moved in proportion")

    # 6. MoveBase (바퀴 로봇). 제자리에서 90도 돌고, 직선으로 2 m 간다. 방향은 가까운 쪽으로 돈다.
    def program_drive():
        yield MoveBase(basetarget(0.0, 0.0, 90.0), v500, fine)
        yield MoveBase(basetarget(0.0, 2.0, 90.0), v500, fine)
        yield MoveBase(basetarget(0.0, 2.0, -180.0), v500, fine)  # 90 → -180은 +90도만 더 돌면 된다 (270도가 아니다)
        yield WaitTime(10.0)

    task = RapidTask(program_drive(), {})
    actual, turned_at, drove_at, yaw_max = np.zeros(3), None, None, 0.0
    for i in range(int(12.0 / DT)):
        task.step(DT, base=actual)
        actual = task.base.copy()
        yaw_max = max(yaw_max, actual[2])
        if turned_at is None and abs(actual[2] - math.pi / 2) < 1e-6:
            turned_at = i * DT
        if drove_at is None and abs(actual[1] - 2.0) < 1e-6:
            drove_at = i * DT
    assert turned_at is not None and abs(turned_at - (math.pi / 2 * TURN_RADIUS / v500 + v500 / BASE_ACCEL)) < 0.1, turned_at
    assert drove_at is not None and abs(drove_at - turned_at - (2.0 / v500 + v500 / BASE_ACCEL)) < 0.1, drove_at
    assert abs(actual[2] - math.pi) < 1e-6 and yaw_max <= math.pi + 1e-6, (actual, yaw_max)
    print(f"PASS MoveBase drive: turned in {turned_at:.2f} s, drove 2 m in {drove_at - turned_at:.2f} s, took the short way round")

    # 7. MoveBase (걷는 로봇). 지령 경로 없이 목표만 내놓고, 실제로 zone 안에 들어와야 다음 목표로 넘어간다.
    def program_walk():
        yield MoveBase(basetarget(3.0, 0.0, 0.0), v500, z500)
        yield MoveBase(basetarget(3.0, 3.0, 0.0), v500, z500)
        yield WaitTime(10.0)

    task = RapidTask(program_walk(), {}, walk=True)
    task.step(DT, base=(0.0, 0.0, 0.0))
    assert np.allclose(task.goal[0], [3.0, 0.0]) and task.goal[1] == v500 and task.base is not None
    task.step(DT, base=(2.4, 0.0, 0.0))  # 아직 0.6 m 남았다
    assert np.allclose(task.goal[0], [3.0, 0.0])
    for _ in range(2):  # 0.5 m 안에 들어왔다. 다음 목표로 넘어간다
        task.step(DT, base=(2.6, 0.0, 0.0))
    assert np.allclose(task.goal[0], [3.0, 3.0])
    for _ in range(2):  # 마지막 목표에 도착하면 걸을 일이 없다
        task.step(DT, base=(3.0, 2.8, 0.0))
    assert task.goal is None
    print("PASS MoveBase walk: exposed each goal in turn and advanced only when the robot got inside the zone")
    print("SELF-TEST PASS")
