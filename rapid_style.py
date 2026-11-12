# ABB RAPID 방식(robtarget·speeddata·zonedata·tooldata·MoveL)으로 쓴 프로그램을 시각별 TCP 목표로 바꿔 주는 층
r"""RobotStudio에서 쓰던 방식 그대로 "좌표 + 속도 + zone + tool"로 동작을 적을 수 있게 해 준다.

Isaac Sim과 무관한 순수 계산이다. 그래서 이 파일만 따로 실행해 시험할 수 있다.

    C:\isaacsim-6.2\python.bat rapid_style.py

RAPID와 이 파일의 대응.

    RAPID                                   이 파일
    ------------------------------------    ----------------------------------------
    CONST robtarget p := [[x,y,z],[q],..];  p = robtarget(x, y, z, rx, ry, rz)
    Offs(p, dx, dy, dz)                     Offs(p, dx, dy, dz)
    v1000, z50, fine                        v1000, z50, fine
    PERS tooldata t := [...];               t = tooldata(x, y, z, rx, ry, rz)
    MoveL p, v1000, z50, t;                 yield MoveL(p, v1000, z50, t)
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
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation

ACCEL = 4.0  # [m/s²] TCP가 속도를 올리고 내리는 한계. RAPID의 AccSet에 해당한다
IN_POSITION = 0.005  # [m] 정지점(fine)에서 실제 TCP가 목표에 이만큼 가까워져야 도착으로 본다

# speeddata. [m/s] TCP 속도. 이름의 숫자는 ABB와 같이 mm/s다.
v100, v200, v500, v1000, v1500, v2000 = 0.1, 0.2, 0.5, 1.0, 1.5, 2.0
# zonedata. [m] 목표에서 이 거리 안에 들어오면 다음 명령으로 넘어간다. fine은 목표에 정확히 서는 정지점이다.
fine, z1, z10, z50, z100 = 0.0, 0.001, 0.01, 0.05, 0.1


# ------------------------------------------------------------------ 데이터 (robtarget, tooldata)
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


def Offs(target, dx, dy, dz):
    """target에서 월드 x·y·z 방향으로 [m] (dx, dy, dz)만큼 옮긴 새 목표를 돌려준다. 방향은 그대로다."""
    return {"pos": target["pos"] + np.array([dx, dy, dz], dtype=float), "rot": target["rot"]}


# ------------------------------------------------------------------ 명령
# 프로그램은 이 명령들을 yield로 하나씩 내놓는다. 여기서는 내용을 묶어 두기만 하고 실행은 RapidTask가 한다.
def MoveL(target, speed, zone, tool):
    """TCP를 target까지 직선으로 보낸다. speed는 [m/s], zone은 [m]이다."""
    return ("MoveL", target, speed, zone, tool)


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

    program  명령을 yield하는 제너레이터 (아래 시험 코드나 factory_line_demo.py의 palletize_program 참고).
    io       출력 신호를 담는 dict. SetDO가 여기에 값을 쓴다.

    step()을 부를 때마다 "지금 TCP가 있어야 할 자세"를 돌려준다. 그 자세로 로봇을 실제로 보내는 일은
    부르는 쪽(IK)이 한다.
    """

    def __init__(self, program, io):
        self.program = program
        self.io = io
        self.command = None  # 지금 실행 중인 명령. None이면 다음 명령을 꺼낸다
        self.pos = None  # [m] 지령 TCP 위치. 프로그램이 "지금 여기 있어야 한다"고 보는 위치다
        self.rot = None  # 지령 TCP 방향
        self.tool = None  # 지금 쓰는 tooldata
        self.speed = 0.0  # [m/s] 지금 TCP 지령 속도
        self.waited = 0.0  # [s] WaitTime에서 지금까지 기다린 시간

    def step(self, dt, link_pos, link_rot):
        """[s] dt만큼 프로그램을 진행하고 (지령 TCP 위치, 지령 TCP 방향, tooldata)를 돌려준다.

        link_pos, link_rot은 로봇 손끝 링크의 현재 실제 자세다. 프로그램을 시작할 때와 tool이 바뀔 때
        "지금 TCP가 어디 있는가"를 알아내는 데만 쓴다.
        """
        for _ in range(20):  # 시간이 안 드는 명령(SetDO, 이미 참인 WaitUntil)은 같은 스텝에 이어서 처리한다
            if self.command is None:
                self.command = next(self.program)
                self.waited = 0.0
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
            elif kind == "MoveL":
                tool = self.command[4]
                if self.pos is None or tool is not self.tool:  # 시작할 때와 tool이 바뀔 때 현재 TCP 자세에서 출발한다
                    self.tool = tool
                    self.pos = link_pos + link_rot.apply(tool["pos"])
                    self.rot = link_rot * tool["rot"]
                if self._move(dt) and self._arrived(link_pos, link_rot):
                    self.command = None
                break
        return self.pos, self.rot, self.tool

    def _arrived(self, link_pos, link_rot):
        """정지점(fine)이면 실제 TCP가 목표에 도착했는지 본다. zone이 있는 목표는 기다리지 않는다.

        지령이 목표에 닿아도 실제 로봇은 조금 늦게 온다. RAPID의 fine처럼 실제로 도착한 뒤에 다음 명령
        (그리퍼 신호 등)으로 넘어가게 한다.
        """
        _, target, _, zone, tool = self.command
        if zone != fine:
            return True
        actual = link_pos + link_rot.apply(tool["pos"])
        return float(np.linalg.norm(actual - target["pos"])) <= IN_POSITION

    def _move(self, dt):
        """MoveL 한 스텝. 지령 TCP를 목표 쪽으로 조금 옮긴다. 이 명령이 끝났으면 True를 돌려준다."""
        _, target, speed, zone, _ = self.command
        to_go = target["pos"] - self.pos
        distance = float(np.linalg.norm(to_go))  # [m] 목표까지 남은 거리
        # 속도 정하기. 가속 한계 안에서 지정 속도까지 올린다.
        velocity = min(speed, self.speed + ACCEL * dt)
        if zone == fine:  # 정지점이면 목표에 가까워질수록 줄여서 정확히 선다
            velocity = min(velocity, math.sqrt(2.0 * ACCEL * distance))
        advance = velocity * dt  # [m] 이번 스텝에 갈 거리
        if distance <= advance:  # 이번 스텝에 목표에 닿는다
            self.pos, self.rot = target["pos"].copy(), target["rot"]
            self.speed = 0.0 if zone == fine else velocity
            return True
        # 위치는 직선으로, 방향은 남은 거리에 비례해서 같이 돌린다. 목표에 닿을 때 방향도 맞는다.
        fraction = advance / distance
        self.pos = self.pos + to_go * fraction
        self.rot = Rotation.from_rotvec((target["rot"] * self.rot.inv()).as_rotvec() * fraction) * self.rot
        self.speed = velocity
        # zone이 있으면 반경 안에 들어온 순간 끝낸다. 멈추지 않고 다음 명령이 이어받는다.
        return zone > 0.0 and distance - advance <= zone


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
            pos, rot, _ = task.step(DT, link_pos, link_rot)
            log.append((i * DT, pos.copy(), task.speed))
            link_pos, link_rot = pos.copy(), rot
        return log, rot

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
        task.step(DT, origin, still)
    assert signals["grip"] == 0  # 조건이 거짓인 동안은 넘어가지 않는다
    gate["open"] = True
    task.step(DT, origin, still)
    assert signals["grip"] == 1
    for _ in range(int(0.2 / DT) + 2):
        task.step(DT, origin, still)
    assert signals["grip"] == 0
    print("PASS io: WaitUntil held the program, SetDO ran in order around WaitTime")

    # 4. 정지점은 실제 로봇이 도착할 때까지 기다린다. 로봇이 안 따라오면 다음 명령(SetDO)으로 넘어가지 않는다.
    signals = {"grip": 0}

    def program_wait_robot():
        yield MoveL(robtarget(0.1, 0, 0, 180, 0, 0), v500, fine, tool0)
        yield SetDO("grip", 1)
        yield WaitTime(10.0)

    task = RapidTask(program_wait_robot(), signals)
    for _ in range(400):  # 2초. 지령은 목표에 닿지만 로봇(손끝 링크)은 원점에 그대로 있다
        task.step(DT, origin, Rotation.from_euler("ZYX", [0, 0, 180], degrees=True))
    assert signals["grip"] == 0
    for _ in range(2):  # 로봇이 도착했다. 첫 스텝에 MoveL이 끝나고 다음 스텝에 SetDO가 실행된다
        task.step(DT, np.array([0.1, 0.0, 0.0]), Rotation.from_euler("ZYX", [0, 0, 180], degrees=True))
    assert signals["grip"] == 1
    print("PASS fine waits for the robot: SetDO ran only after the actual TCP reached the target")
    print("SELF-TEST PASS")
