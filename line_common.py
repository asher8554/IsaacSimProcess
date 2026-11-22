# Isaac Sim 노드와 RAPID 컨트롤러 노드가 같이 쓰는 라인 배치·공구·ROS 2 설정과 메시지 변환
r"""두 프로세스가 같은 숫자를 쓰도록 한곳에 모은 파일이다.

    factory_line_demo.py   Isaac Sim 노드 (/isaac_sim). 로봇 본체, 관절 구동, IK, 보행 정책, 센서
    rapid_controller.py    RAPID 컨트롤러 노드 (/rapid_controller). RAPID 스타일 프로그램 4개를 돌린다

두 노드는 서로를 import하지 않는다. ROS 2 topic·service로만 대화하고, 배치 숫자와 tooldata만 이 파일에서 같이 읽는다.
실제 공장에 비유하면 "셀 도면과 공구 사양서"다. 컨트롤러와 로봇이 같은 도면을 보고 있어야 좌표가 맞는다.

내 셀로 바꿀 때는 이 파일의 숫자부터 내 도면 값으로 고친다. 두 프로세스가 같이 바뀐다.
"""

import math
import os
import sys

from rapid_style import tooldata

# ------------------------------------------------------------------ ROS 2 실행 환경
# RMW는 ROS 2가 실제로 메시지를 실어 나르는 통신 구현이다. 두 프로세스가 같은 RMW를 써야 서로를 찾는다.
# Isaac Sim의 python.bat은 기본값을 rmw_zenoh_cpp로 정한다. Zenoh는 라우터(rmw_zenohd)를 따로 켜야 다른 프로세스를
# 찾는다. 이 예시는 라우터 없이 바로 통하는 FastDDS로 고정한다. Zenoh를 쓰고 싶으면 이 값을 바꾸고 라우터를 먼저 켠다.
RMW = "rmw_fastrtps_cpp"
os.environ["RMW_IMPLEMENTATION"] = RMW

# Isaac Sim 설치본에 ROS 2 Jazzy의 rclpy와 표준 메시지가 들어 있다. ROS 2를 따로 설치하지 않고 이것을 쓴다.
# sys.path 맨 뒤에 붙인다. 앞에 두면 그 폴더 안의 numpy가 Isaac Sim의 numpy를 가려 DLL 오류가 난다.
# ISAAC_PATH는 python.bat이 정해 주는 Isaac Sim 설치 폴더다. python.bat으로 실행해야 이 값이 있다.
_bundled = os.path.join(os.environ.get("ISAAC_PATH", r"C:\isaacsim-6.2"), "exts", "isaacsim.ros2.core", "jazzy", "rclpy")
if _bundled not in sys.path:
    sys.path.append(_bundled)

import numpy as np  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

# ------------------------------------------------------------------ 라인 배치
# 주석 맨 앞 대괄호가 그 값의 단위다. [m] 미터, [s] 초, [m/s] 초당 미터, [rad] 라디안(180도 = 3.1416 rad).
# 좌표는 월드 기준이다. x·y가 바닥 평면이고 z가 위쪽이다.
TAKT = 8.0  # [s] 택트 타임. 박스 1개가 나오는 간격
BELT_V = 0.225  # [m/s] 컨베이어 위에서 박스가 움직이는 속도
BELT_Z = 0.77  # [m] 바닥에서 컨베이어 롤러 윗면까지 높이 (ConveyorBelt_A05 실측)
BOX = 0.35  # [m] 박스 한 변의 길이 (정육면체)
X_IN, X_OUT = -3.7, 1.7  # [m] 박스가 벨트에 올라오는 x, 집히는 x. 둘 사이 거리 = 3 * TAKT * BELT_V
GAP = 0.02  # [m] 그리퍼 끝과 박스 윗면 사이 틈
PALLET_TOP = 0.143  # [m] 바닥에서 팔레트 윗면까지 높이 (pallet.usd 실측)
PALLET_OUT = (3.2, -1.5)  # [m] (x, y) 로봇 B가 박스를 쌓는 팔레트의 중심
PALLET_BUF = (-3.0, -1.5)  # [m] (x, y) AMMR이 다녀오는 출하 버퍼 팔레트의 중심
AMMR_HOME = (PALLET_OUT[0], -3.0)  # [m] (x, y) AMMR을 처음 놓는 위치. AMMR 베이스 관절값 (0, 0)이 이 점이다
SLOT_X = (-0.4, 0.0, 0.4)  # [m] 팔레트 중심에서 박스 3칸 각각까지의 x 거리
SCAN_START = (-1.45, 0.0, BELT_Z + BOX + 0.10)  # [m] (x, y, z) KUKA가 검사를 시작하는 점. 박스 윗면에서 10 cm 위
SCAN_LEN = 0.90  # [m] KUKA가 박스와 나란히 가는 거리. 벨트 방향(+x)이다
TUCK = [0.0, -1.0, 0.0, -2.4, 0.0, 1.6, 0.785]  # [rad] AMMR 팔 7축. 접은 자세
REACH = [0.0, 0.75, 0.0, -1.3, 0.0, 2.05, 0.785]  # [rad] AMMR 팔 7축. 앞으로 뻗은 자세
H1_WAYPOINTS = [(4.0, 4.4), (4.0, 6.0), (-4.0, 6.0), (-4.0, 4.4)]  # [m] (x, y) H1 순찰 지점 4곳. 반시계 순서로 돈다

# ------------------------------------------------------------------ tooldata
# 손끝 링크에서 본 TCP의 [m] 위치 (x, y, z)와 [deg] 방향 (rx, ry, rz)이다. TCP의 Z축이 공구가 뻗는 방향이다.
# 컨트롤러는 MoveL에 쓰고, Isaac Sim은 IK에서 TCP를 계산할 때 쓴다. 지령 메시지에는 이 사전의 이름이 실려 간다.
# 공구를 새로 만들면 여기에 이름과 함께 추가한다. 두 프로세스가 같은 이름으로 같은 값을 찾는다.
TOOLS = {
    # Kawasaki. 손끝 링크(gripper_center)의 +Z가 공구 방향이다. 그 방향으로 GAP만큼 나간 점을 TCP로 잡았다.
    "tGripper": tooldata(0.0, 0.0, GAP, 0.0, 0.0, 0.0),
    # KUKA. 손끝 링크(tool0)는 +X가 공구 방향이다. y축으로 90도 돌려 TCP의 Z축을 그 방향에 맞춘다.
    "tScanner": tooldata(0.0, 0.0, 0.0, 0.0, 90.0, 0.0),
}


# ------------------------------------------------------------------ 메시지 변환
# ROS 2 메시지는 정해진 모양(타입)이 있다. 아래 함수들은 numpy·scipy 값과 메시지 사이를 바꿔 준다.
def stamp(seconds):
    """[s] 시뮬레이션 시각을 메시지 머리말의 시각(초 + 나노초)으로 바꾼다."""
    sec = math.floor(seconds)
    return Time(sec=int(sec), nanosec=int((seconds - sec) * 1e9))


def seconds(time_msg):
    """메시지 머리말의 시각을 [s] 실수로 바꾼다."""
    return time_msg.sec + time_msg.nanosec * 1e-9


def to_transform(t, pos, rot, child):
    """[s] 시각 t에 world 기준으로 child 좌표계가 [m] pos, rot 자세에 있다는 TransformStamped 메시지를 만든다.

    ROS 2에서 "어느 좌표계가 어디에 있다"를 나타내는 표준 메시지다. child_frame_id에 좌표계 이름을 적는다.
    이 예시에서는 손끝 링크(kuka/tool0 등)의 실제 자세와 TCP 지령(tool 이름)을 실어 보낸다.
    """
    msg = TransformStamped()
    msg.header.stamp = stamp(t)
    msg.header.frame_id = "world"
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = (float(v) for v in pos)
    q = msg.transform.rotation
    q.x, q.y, q.z, q.w = (float(v) for v in rot.as_quat())  # scipy 쿼터니언 순서는 (x, y, z, w)이고 ROS도 같다
    return msg


def from_transform(msg):
    """TransformStamped 메시지를 ([m] 위치 numpy 배열, scipy Rotation)으로 바꾼다."""
    p, q = msg.transform.translation, msg.transform.rotation
    return np.array([p.x, p.y, p.z]), Rotation.from_quat([q.x, q.y, q.z, q.w])
