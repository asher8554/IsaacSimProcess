# MoveIt 없이 Isaac Sim의 FollowJointTrajectory 액션에 손으로 만든 궤적을 보내는 시험 클라이언트
r"""실행 (Windows, isaac_panda_sim.py가 떠 있는 상태에서 다른 터미널):
    C:\isaacsim-6.2\python.bat moveit_lab\send_trajectory.py

무엇을 보여 주나.
    MoveIt이 하는 마지막 일은 "시간이 붙은 관절 궤적"을 이 액션으로 보내는 것뿐이다.
    이 스크립트는 그 궤적을 사람이 직접 적어서 보낸다. 경로 계획·IK·충돌 검사가 없으므로
    관절 각도와 시각을 사람이 다 정해야 한다. RAPID의 MoveAbsJ를 시간표로 적는 것과 비슷하다.
    MoveIt을 쓰면 이 부분(어떤 각도를 어떤 시각에)을 MoveIt이 계산해 준다.

성공하면 "result: error_code 0 (reached ...)"가 두 번 찍히고 종료 코드 0으로 끝난다.
"""

import os
import sys

os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
# Isaac Sim 설치본에 들어 있는 ROS 2 Jazzy rclpy를 쓴다. sys.path 맨 뒤에 붙여야 numpy 충돌이 없다.
sys.path.append(os.path.join(os.environ.get("ISAAC_PATH", r"C:\isaacsim-6.2"), "exts", "isaacsim.ros2.core", "jazzy", "rclpy"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

from lab_common import ARM_JOINTS, READY, SIDE, TRAJECTORY_ACTION  # noqa: E402


def trajectory(start, goal, seconds):
    """start에서 goal까지 [s] seconds 동안 가는 궤적. 중간 점 없이 두 점만 적는다 (속도 0에서 출발·정지)."""
    return JointTrajectory(
        joint_names=ARM_JOINTS,
        points=[
            JointTrajectoryPoint(positions=start, velocities=[0.0] * 7, time_from_start=Duration(sec=0)),
            JointTrajectoryPoint(positions=goal, velocities=[0.0] * 7, time_from_start=Duration(sec=int(seconds))),
        ],
    )


def send(node, client, traj):
    """궤적 하나를 보내고 끝날 때까지 기다린다. 성공하면 True."""
    goal = client.send_goal_async(FollowJointTrajectory.Goal(trajectory=traj))
    rclpy.spin_until_future_complete(node, goal, timeout_sec=10.0)
    handle = goal.result()
    if handle is None or not handle.accepted:
        print("goal rejected or no answer")
        return False
    done = handle.get_result_async()
    rclpy.spin_until_future_complete(node, done, timeout_sec=30.0)
    if done.result() is None:
        print("no result within 30 s")
        return False
    result = done.result().result
    print(f"result: error_code {result.error_code} ({result.error_string})")
    return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL


def main():
    rclpy.init()
    node = rclpy.create_node("send_trajectory")
    client = ActionClient(node, FollowJointTrajectory, TRAJECTORY_ACTION)
    if not client.wait_for_server(timeout_sec=30.0):
        print(f"{TRAJECTORY_ACTION} 액션 서버를 못 찾았다. isaac_panda_sim.py가 떠 있는지 확인한다.")
        sys.exit(1)
    # 시뮬레이터는 ready 자세에서 시작한다. ready → SIDE (3초) → ready (3초).
    ok = send(node, client, trajectory(READY, SIDE, 3)) and send(node, client, trajectory(SIDE, READY, 3))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
