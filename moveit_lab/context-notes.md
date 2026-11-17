# MoveIt 실습 결정 기록

## 2026-11-17

- **기존 라인 데모와 분리했다.** `moveit_lab/` 폴더에 따로 둔다. 라인 데모는 RAPID 방식 비교 대상으로 그대로 남긴다. 작업 당시 루트에는 다른 작업(ROS 2 노드 분리)의 커밋 안 된 변경이 있어서 섞지 않으려는 이유도 있었다.
- **로봇은 Franka Panda.** MoveIt 공식 설정(`moveit_resources_panda_moveit_config`)이 apt에 있고, Isaac Sim 에셋도 있으며, 관절 이름이 두 쪽에서 같다. 다른 로봇은 MoveIt 설정 패키지를 따로 만들어야 한다.
- **MoveIt은 WSL, Isaac Sim은 Windows.** MoveIt은 Windows 지원이 사실상 없다. WSL NAT 기본 설정에서 FastDDS가 양방향으로 통했다 (시험 스크립트로 확인).
- **실행 연결은 Isaac Sim 안의 FollowJointTrajectory 액션 서버.** 공식 방식(`topic_based_ros2_control`)은 Jazzy apt에 패키지가 없어 소스 빌드가 필요하다. Isaac Sim 내장 rclpy에 `control_msgs`가 있어서 액션 서버를 직접 열었다. Notion "12 MoveIt + MuJoCo" 강의의 브리지 방식(액션 서버 + Hermite 보간)과 같은 구조다.
- **`allowed_start_tolerance`를 0.05 rad로 올렸다.** Isaac Sim Panda 관절 구동기의 정지 오차가 약 0.014 rad로 Panda 기본값 0.01보다 크다 (send_trajectory 시험 결과).
- **`use_sim_time`을 켰다.** 헤드리스 Isaac Sim이 실시간의 0.41배로 돈다. 벽시계를 쓰면 MoveIt의 실행 시간 감시가 중단시킨다.
- **예제는 move_group의 액션·서비스를 직접 부른다.** `moveit_py`는 설정이 더 필요하고, 사용자가 Notion 강의에서 본 `joint_goal.py`와 같은 방식이라 연결이 쉽다.
- **Pilz를 계획기에 포함했다.** PTP·LIN·CIRC가 RAPID MoveJ·MoveL·MoveC에 바로 대응한다.

### MoveIt 연동 실행에서 고친 것 (2026-11-17)

- **실행 시간 감시가 벽시계 기준이다.** `use_sim_time`을 켜도 4.31 s 궤적이 벽시계 5.67 s 상한에 걸려 `TIMED_OUT`이 났다(헤드리스 0.41배). `allowed_execution_duration_scaling` 4.0, `allowed_goal_duration_margin` 2.0으로 올렸다.
- **Pilz 파이프라인에 `ValidateSolution`이 없다.** Panda 설정의 pilz yaml에 response_adapters가 없어 LIN이 기둥을 뚫어도 SUCCESS로 계획되고 실제로 부딪혔다. launch에서 추가했다. 추가 후 PTP 직선도 기둥을 쓸면 `INVALID_MOTION_PLAN`으로 거부된다(예제 4 첫 이동을 OMPL로 바꾼 이유).
- **SRDF 이름 자세가 쓸모가 제한적이다.** `transport`는 MoveIt 모델에서 link5·link7 자기 충돌(`/check_state_validity`로 확인), `extended`는 j4 = 0이 Isaac Franka 상한(약 −0.07 rad) 밖이라 실행 중 0.063 rad 남기고 abort. 예제 1은 직접 정의한 `SIDE` 자세를 쓴다.
- **공구가 아래를 보는 자세의 도달 높이.** x 0.45 m에서 TCP z 0.65 m까지만 IK 해가 있다(계획만 시켜 탐색). 기둥을 0.45 m로 낮추고 경유점을 0.55 m에 뒀다.
- **move_group 결과의 `planning_time`은 항상 0이었다.** 출력과 문서에서 계획 시간 비교를 뺐다.
- **Isaac Franka 관절 구동기 게인은 kp 22918, kd 4584.** 궤적 끝에서 오차가 0.02 rad 아래로 내려오는 데 문제없어 바꾸지 않았다.
