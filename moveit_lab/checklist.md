# MoveIt 실습 체크리스트

- [x] WSL Ubuntu 24.04 + ROS 2 Jazzy 설치 상태 확인 (MoveIt 미설치, sudo 비밀번호 필요)
- [x] Windows Isaac Sim ↔ WSL ROS 2 FastDDS 양방향 통신 확인
- [x] Panda MoveIt 설정 확인 (컨트롤러 `panda_arm_controller`, Pilz 포함, `topic_based_ros2_control` apt 없음)
- [x] `lab_common.py` 공용 숫자
- [x] `isaac_panda_sim.py` 장면 + FollowJointTrajectory 액션 서버
- [x] `send_trajectory.py`로 액션 서버 검증 (궤적 2개 성공, SELF-CHECK PASS)
- [x] `wsl/moveit_isaac.launch.py`
- [x] 예제 1~4와 `moveit_client.py`
- [x] `README.md` 학습 안내
- [x] WSL에 MoveIt 설치 (사용자가 sudo 입력, ros-jazzy-moveit 2.12.4)
- [x] launch + 예제 1~4 실행 검증 (모두 PASS, ex3 --no-scene은 의도대로 CONTROL_FAILED)
- [x] 실행 중 발견한 문제 수정 (실행 시간 감시, Pilz 충돌 검사, 도달 높이, 자세 선택)
- [x] GUI 화면 확인 (Isaac Sim 창, RViz) → docs/isaac_moveit_rviz.png
- [x] 커밋
