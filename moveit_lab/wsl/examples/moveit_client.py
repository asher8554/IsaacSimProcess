# 예제들이 같이 쓰는 MoveIt 요청 도우미. move_group에 액션·서비스로 "어디로 가라"를 보낸다
r"""WSL에서 python3로 실행하는 예제들이 import한다. MoveIt 본체(move_group)는 launch 파일로 먼저 띄워 둔다.

MoveIt에 일을 시키는 방법은 여러 가지다. 이 파일은 가장 바탕이 되는 방법, 즉 move_group의 ROS 2 인터페이스를
직접 부르는 방법을 쓴다. C++ MoveGroupInterface나 Python moveit_py도 결국 같은 액션·서비스를 부른다.

    /move_action                MoveGroup 액션        목표(관절값·TCP 자세)를 주면 계획하고, 원하면 실행까지 한다
    /execute_trajectory         ExecuteTrajectory 액션  이미 계획된 궤적을 실행한다
    /apply_planning_scene       ApplyPlanningScene 서비스  장애물을 Planning Scene에 넣고 뺀다

RAPID와 대응
    move_joints(...)          MoveAbsJ. 관절값 목표. 경로는 OMPL이 장애물을 피해 찾는다
    move_tcp(..., "ompl")     MoveJ에 가깝다. TCP 목표를 IK로 풀고 경로는 자유롭게 찾는다
    move_tcp(..., "PTP")      MoveJ. Pilz 계획기. 관절 공간 직선(축들이 같이 출발·같이 도착)
    move_tcp(..., "LIN")      MoveL. Pilz 계획기. TCP가 직선으로 간다. 장애물은 피하지 않고 막히면 실패한다
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

# lab_common.py는 이 파일의 두 단계 위 폴더(moveit_lab)에 있다. Isaac Sim 쪽과 같은 숫자를 읽는다.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from lab_common import ARM_JOINTS, HAND_LINK, OBSTACLE, TCP_Z  # noqa: E402, F401 (ARM_JOINTS는 예제가 가져다 쓴다)

GROUP = "panda_arm"  # SRDF의 계획 그룹 이름. 관절 7개 묶음이다
FRAME = "world"  # 목표 좌표의 기준 좌표계. launch 파일이 world = panda_link0 = Isaac Sim 월드 원점으로 맞춰 두었다
# MoveIt 결과 코드 중 자주 보는 것. 전체 목록은 moveit_msgs/msg/MoveItErrorCodes에 있다.
ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items() if k.isupper() and isinstance(v, int)}


# ------------------------------------------------------------------ 자세 계산 (scipy 없이)
def quat_from_rpy(rx, ry, rz):
    """[deg] 고정축 x→y→z 순서 회전 (RAPID의 OrientZYX와 같은 뜻)을 쿼터니언 (x, y, z, w)로 바꾼다."""
    hx, hy, hz = (math.radians(a) / 2 for a in (rx, ry, rz))
    cx, sx, cy, sy, cz, sz = math.cos(hx), math.sin(hx), math.cos(hy), math.sin(hy), math.cos(hz), math.sin(hz)
    return (
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
        cx * cy * cz + sx * sy * sz,
    )


def rotate(q, v):
    """쿼터니언 q (x, y, z, w)로 벡터 v를 돌린다."""
    x, y, z, w = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def hand_pose(tcp, rpy=(180.0, 0.0, 0.0)):
    """TCP 목표 [m] (x, y, z), [deg] (rx, ry, rz)를 MoveIt이 아는 손 링크(panda_hand)의 Pose로 바꾼다.

    RAPID에서는 tooldata를 주면 컨트롤러가 이 계산을 해 준다. MoveIt의 Panda 모델에는 TCP 링크가 없어서
    손 링크 기준으로 목표를 줘야 한다. TCP는 손 링크 +Z 방향으로 TCP_Z만큼 나간 점이다.
    기본 방향 (180, 0, 0)은 공구가 바닥을 보는 자세다 (rapid_style.py의 robtarget과 같은 약속).
    """
    q = quat_from_rpy(*rpy)
    off = rotate(q, (0.0, 0.0, TCP_Z))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (tcp[i] - off[i] for i in range(3))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose


def named_pose(name):
    """SRDF에 이름으로 적힌 자세(group_state)를 [rad] {관절 이름: 값}으로 읽는다. 예: ready, transport, extended."""
    srdf = Path(get_package_share_directory("moveit_resources_panda_moveit_config")) / "config" / "panda.srdf"
    state = ET.parse(srdf).find(f"group_state[@group='{GROUP}'][@name='{name}']")
    if state is None:
        raise SystemExit(f"SRDF에 '{name}' 자세가 없다")
    return {j.get("name"): float(j.get("value")) for j in state.findall("joint")}


# ------------------------------------------------------------------ MoveIt 클라이언트
class MoveItClient:
    """move_group에 요청을 보내는 작은 클라이언트. 예제마다 하나씩 만든다."""

    def __init__(self, name, pillar=True):
        """name은 ROS 2 노드 이름이다. pillar=True면 시작할 때 기둥을 Planning Scene에 넣는다.

        Isaac Sim 장면에는 기둥이 항상 서 있다. 그래서 예제는 기본으로 MoveIt에도 기둥을 알려 준다.
        예제 3의 --no-scene만 pillar=False로 MoveIt이 기둥을 모르는 상태를 만든다.
        """
        rclpy.init()
        # use_sim_time: 이 노드의 시계도 Isaac Sim의 /clock을 따른다. 요청 시각이 move_group과 같은 시계가 된다.
        self.node = rclpy.create_node(name, parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.move = ActionClient(self.node, MoveGroup, "/move_action")
        self.scene = self.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not self.move.wait_for_server(timeout_sec=30.0):
            raise SystemExit("/move_action을 못 찾았다. WSL에서 moveit_isaac.launch.py를 먼저 띄운다.")
        # 기본 속도·가속 배율. 1.0이면 joint_limits.yaml 한계까지 쓴다. RAPID의 VelSet과 같은 역할이다.
        self.velocity = 0.3
        self.acceleration = 0.3
        if pillar:
            self.add_box(OBSTACLE["name"], OBSTACLE["center"], OBSTACLE["size"])
            print(f"Planning Scene: '{OBSTACLE['name']}' 등록")
        else:
            self.remove_box(OBSTACLE["name"])
            print(f"Planning Scene: '{OBSTACLE['name']}' 제거 (MoveIt은 기둥을 모른다)")

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    # --- 공통: MoveGroup 액션 한 번 보내기
    def _send(self, constraints, pipeline="ompl", planner="", plan_only=False):
        """목표 조건 하나로 계획(+실행)한다. 성공하면 True. 결과 코드와 궤적 길이를 출력한다."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.pipeline_id = pipeline  # "ompl" 또는 "pilz_industrial_motion_planner"
        req.planner_id = planner  # Pilz면 "PTP"·"LIN"·"CIRC". OMPL은 비워 두면 기본 계획기(RRTConnect)
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0  # [s] 계획에 쓸 수 있는 최대 시간
        req.max_velocity_scaling_factor = self.velocity
        req.max_acceleration_scaling_factor = self.acceleration
        req.start_state.is_diff = True  # 시작 자세 = 지금 /joint_states 자세
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = plan_only  # True면 계획만 하고 로봇은 움직이지 않는다 (RViz에 경로만 보인다)
        sent = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent)
        handle = sent.result()
        if not handle.accepted:
            print("  move_group이 요청을 거부했다")
            return False
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, done)
        result = done.result().result
        code = result.error_code.val
        traj = result.planned_trajectory.joint_trajectory
        length = traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9 if traj.points else 0.0
        print(f"  결과 {ERROR_NAMES.get(code, code)} | 궤적 점 {len(traj.points)}개, {length:.2f} s")
        return code == MoveItErrorCodes.SUCCESS

    # --- 목표 종류별 함수
    def move_joints(self, joints, plan_only=False):
        """[rad] {관절 이름: 값} 목표로 간다. MoveAbsJ에 해당한다. 허용 오차는 관절마다 ±0.01 rad."""
        c = Constraints()
        for name, value in joints.items():
            c.joint_constraints.append(
                JointConstraint(joint_name=name, position=value, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            )
        return self._send(c, plan_only=plan_only)

    def move_tcp(self, tcp, rpy=(180.0, 0.0, 0.0), planner="ompl", plan_only=False):
        """TCP를 [m] (x, y, z), [deg] (rx, ry, rz)로 보낸다. planner는 'ompl', 'PTP', 'LIN' 중 하나다.

        목표 조건은 두 개다. 손 링크 원점이 지름 2 mm 구 안에 들어와야 하고(위치), 방향이 축마다 0.01 rad 안이어야 한다.
        "정확히 이 점"이 아니라 "이 범위 안"이라고 주는 것이 MoveIt 방식이다. 범위가 0이면 IK가 해를 못 찾는다.
        """
        pose = hand_pose(tcp, rpy)
        c = Constraints()
        pc = PositionConstraint(link_name=HAND_LINK, weight=1.0)
        pc.header.frame_id = FRAME
        pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])]
        pc.constraint_region.primitive_poses = [Pose(position=pose.position)]
        oc = OrientationConstraint(link_name=HAND_LINK, orientation=pose.orientation, weight=1.0)
        oc.header.frame_id = FRAME
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = 0.01
        c.position_constraints, c.orientation_constraints = [pc], [oc]
        if planner == "ompl":
            return self._send(c, plan_only=plan_only)
        return self._send(c, pipeline="pilz_industrial_motion_planner", planner=planner, plan_only=plan_only)

    def add_box(self, name, center, size):
        """[m] center (x, y, z)에 [m] size (x, y, z) 상자를 Planning Scene에 넣는다. MoveIt은 이것을 피해 계획한다."""
        box = CollisionObject(id=name, operation=CollisionObject.ADD)
        box.header.frame_id = FRAME
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in center)
        pose.orientation.w = 1.0
        box.primitive_poses = [pose]
        return self._apply(box)

    def remove_box(self, name):
        """Planning Scene에서 상자를 뺀다. 없는 이름이어도 오류가 아니다."""
        box = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        box.header.frame_id = FRAME
        return self._apply(box)

    def _apply(self, obj):
        if not self.scene.wait_for_service(timeout_sec=10.0):
            raise SystemExit("/apply_planning_scene 서비스를 못 찾았다")
        scene = PlanningScene(is_diff=True)  # is_diff: 지금 장면에 이 변경만 더한다
        scene.world.collision_objects = [obj]
        done = self.scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self.node, done)
        return done.result().success


def run(steps):
    """(설명, 함수) 목록을 차례로 실행하고, 하나라도 실패하면 종료 코드 1로 끝낸다."""
    ok = True
    for text, step in steps:
        print(f"▶ {text}")
        ok = step() and ok
    print("EXAMPLE", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
