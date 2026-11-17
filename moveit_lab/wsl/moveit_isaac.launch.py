# WSL에서 MoveIt(move_group)·RViz·TF를 띄우고 실행은 Windows의 Isaac Sim 액션 서버에 맡기는 launch 파일
r"""실행 (WSL Ubuntu 터미널):
    source /opt/ros/jazzy/setup.bash
    ros2 launch /mnt/e/Github/IsaacSimProcess/moveit_lab/wsl/moveit_isaac.launch.py

MoveIt 공식 Panda 데모(moveit_resources_panda_moveit_config의 demo.launch.py)에서 두 가지를 바꿨다.
  1. ros2_control(가짜 하드웨어)을 띄우지 않는다. 궤적 실행은 Isaac Sim이 연 액션 서버
     /panda_arm_controller/follow_joint_trajectory가 맡는다. 이름이 MoveIt 컨트롤러 설정과 같아서 그대로 연결된다.
  2. use_sim_time을 켠다. 모든 노드가 Isaac Sim이 보내는 /clock을 시계로 쓴다.
  3. 실행 시간 감시 배율을 4배로 키운다. Isaac Sim이 실시간보다 느리게 돌아도(헤드리스 약 0.4배) MoveIt이
     "실행이 너무 오래 걸린다"며 취소하지 않게 한다 (아래 start_tolerance 주석 참고).

띄우는 노드
  move_group              MoveIt 본체. IK, 경로 계획(OMPL·Pilz), 충돌 검사, 시간 붙이기, 실행 관리
  robot_state_publisher   /joint_states를 받아 링크마다 TF를 낸다. RViz가 로봇 모양을 그릴 때 쓴다
  static_transform_publisher   world → panda_link0. 로봇을 월드 원점에 둔다 (Isaac Sim 장면과 같다)
  rviz2                   MotionPlanning 패널. 마우스로 목표를 잡고 Plan·Execute를 누를 수 있다

인자
  rviz:=false   RViz 없이 띄운다 (예제 스크립트만 돌릴 때)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt 설정 묶음. "moveit_resources_panda"라고 주면 moveit_resources_panda_moveit_config 패키지의 config/를 읽는다.
    # 내 로봇이면 MoveIt Setup Assistant로 만든 내 설정 패키지 이름을 넣는다.
    config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")  # URDF: 링크·관절·충돌 모양
        .robot_description_semantic(file_path="config/panda.srdf")  # SRDF: 그룹·이름 자세·충돌 제외 쌍
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # 궤적을 보낼 컨트롤러(액션) 이름
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        # 경로 계획기 두 가지. ompl = 장애물을 피하는 샘플링 계획기(기본), pilz = PTP·LIN·CIRC 산업용 명령
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    sim_time = {"use_sim_time": True}
    # [rad] 궤적 첫 점과 지금 실제 관절의 허용 차이. Panda 기본값 0.01은 Isaac Sim 관절 구동기의 정지 오차(약 0.014)보다
    # 작아서 두 번째 동작부터 "start point deviates" 오류로 실행을 거부한다. 실제 로봇 드라이버도 이 값을 조정한다.
    # 실행 시간 감시. MoveIt은 "궤적 길이 × 배율 + 여유"를 넘기면 실행을 취소한다(TIMED_OUT).
    # 이 감시는 use_sim_time과 상관없이 벽시계로 잰다(2026-11-17 실측). 헤드리스 Isaac Sim은 실시간의 약 0.4배라
    # Panda 기본값(1.2배 + 0.5 s)이면 4.3초 궤적이 5.7초 만에 취소된다. 배율을 4배로 넉넉히 준다.
    # 실제 로봇이면 1.2배 근처로 되돌린다. 느린 로봇을 늦게 알아채는 것이 이 값을 키운 대가다.
    start_tolerance = {
        "trajectory_execution.allowed_start_tolerance": 0.05,
        "trajectory_execution.allowed_execution_duration_scaling": 4.0,
        "trajectory_execution.allowed_goal_duration_margin": 2.0,
    }
    # Pilz 계획 결과 검사. Panda 설정의 pilz yaml에는 response_adapters가 없어서, LIN이 기둥을 뚫어도 "성공"으로
    # 계획되고 실제로 부딪혔다(2026-11-17 실측). ValidateSolution이 계획된 궤적 전체를 충돌 검사한다.
    # DisplayMotionPath는 RViz에 계획된 손끝 경로를 그린다. OMPL 쪽 yaml에는 원래 둘 다 들어 있다.
    pilz_checks = {
        "pilz_industrial_motion_planner.response_adapters": [
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ]
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config.to_dict(), sim_time, start_tolerance, pilz_checks],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", get_package_share_directory("moveit_resources_panda_moveit_config") + "/launch/moveit.rviz"],
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.planning_pipelines,
            config.robot_description_kinematics,
            config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
        parameters=[sim_time],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[config.robot_description, sim_time],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="RViz MotionPlanning 창을 띄운다"),
            world_tf,
            robot_state_publisher,
            move_group,
            rviz,
        ]
    )
