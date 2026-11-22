---
name: ros2-architecture
description: Isaac Sim 노드와 RAPID 컨트롤러 노드의 분리 구조와 통신 규칙
updated: 2026-11-22
---

## 결정
- `/isaac_sim`(factory_line_demo.py)은 로봇 본체·IK·관절 구동·보행 정책·센서, `/rapid_controller`(rapid_controller.py)는 RAPID 프로그램 4개를 돌린다. 외부 PC가 EGM·RSI처럼 지령을 스트리밍하는 방식이다 (사용자 요청 2026-11-17).
- 상태·입력 신호는 Topic, `SetDO`는 Service(`std_srvs/SetBool`). `MoveL`은 Action이 아니다. 보간을 컨트롤러가 하고 매 스텝 지령만 보낸다. Action으로 바꾸려면 Isaac 쪽이 보간하는 구조로 바꿔야 한다.
- 메시지는 번들 표준 타입만 쓴다(Windows에서 인터페이스 패키지를 빌드하지 않기 위해). TCP 지령은 `TransformStamped`, `child_frame_id`에 tool 이름. tool 값은 `line_common.TOOLS`에서 두 프로세스가 같이 찾는다.
- RMW는 `rmw_fastrtps_cpp` 고정. `python.bat` 기본값 `rmw_zenoh_cpp`는 라우터 `rmw_zenohd`가 따로 필요하다.
- 번들 rclpy 폴더(`exts/isaacsim.ros2.core/jazzy/rclpy`)는 `sys.path` 끝에 붙인다. 앞에 두면 그 안의 numpy가 Isaac numpy를 가려 DLL 오류.
- 시계는 `/clock`. 컨트롤러는 받을 때마다 그 시간차로 진행하고, 시계가 뒤로 가면 프로그램을 처음부터 다시 만든다. 시뮬레이터는 자기 시계보다 미래 시각 지령을 버린다.
- 받기 반복은 처리한 메시지가 없으면 멈춘다. 지령은 항상 한 스텝(5 ms) 늦게 적용된다 (2026-11-22).
- 데모가 컨트롤러를 `--child`, `stdin=PIPE`로 띄우고 컨트롤러는 stdin EOF에서 `os._exit(0)`. 부모가 어떻게 끝나도 따라 끝난다 (2026-11-22).

## 이력
- 이전: `atexit.register(controller.terminate)` — Kit SIGINT 처리기가 atexit 전에 프로세스를 끝내 Ctrl+C에서 실패 (2026-11-22, log/2026-11.md)
- 이전: ROS 2 없음, 한 프로세스 (2026-11-12)

출처: log/2026-11.md `2026-11-17 ROS 2 노드 분리`, `2026-11-22 검토 수정`
