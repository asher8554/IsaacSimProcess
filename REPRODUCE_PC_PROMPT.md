# 다른 PC에서 IsaacSimProcess 재현하기

원본 PC의 `IsaacSimProcess` 작업 폴더를 대상 PC로 복사한다. `.git`, `out/`, `__pycache__/`, `.venv/`는 복사 대상에서 빼도 된다. Git에 아직 등록하지 않은 `line_common.py`, `rapid_controller.py`, `tests/test_rapid_controller.py`와 `moveit_lab/`를 포함한다. 2026-11-20 확인 기준으로 GitHub의 `main`만 클론하면 이 작업 사본과 달라진다. MoveIt 실습은 로컬 커밋 `d869835`에 있고, 공장 데모의 ROS 2 분리 코드는 로컬 작업 폴더에 있다.

대상 PC에서 복사한 폴더를 Codex 또는 Claude Code로 열고 아래 인용문을 입력한다.

> `REPRODUCE_PC_PROMPT.md`의 「대상 PC 에이전트에게」부터 읽고, 이 PC에 이미 설치된 Isaac Sim으로 공장 라인 데모와 Panda MoveIt 실습을 재현해 줘. 환경 확인부터 실행, 검증, 오류 해결까지 진행하고 결과를 한국어로 알려 줘.

---

## 대상 PC 에이전트에게

복사한 소스를 기준으로 Windows의 Isaac Sim 6.2에서 **공장 자동화 라인 데모**와 **Panda MoveIt 실습**을 각각 재현해 줘. 계획 설명에서 멈추지 말고 터미널로 직접 실행해 로그와 화면을 확인해 줘. 이번 범위는 기존 시뮬레이션이며 장면·로봇·제어 로직 변경과 실물 로봇 제어는 제외한다. 경로와 버전은 이 PC에서 조사해 사용하고, 이미 설치된 환경을 우선한다.

### 1. 소스와 실행 환경 확인

- 저장소 루트에서 `README.md`, `moveit_lab/README.md`와 실행 파일을 읽는다. `factory_line_demo.py`, `rapid_style.py`, `rapid_controller.py`, `line_common.py`, `tests/`, `moveit_lab/wsl/`이 있는지 확인한다. 파일이 모자라면 해당 경로를 알려 주고, 확보된 파일로 검증 가능한 작업을 계속한다. 원격 사본으로 로컬 파일을 덮어쓰지 않는다.
- Isaac Sim 설치 폴더의 `VERSION`, `python.bat`, 번들 ROS 2 Jazzy 경로 `exts/isaacsim.ros2.core/jazzy`를 확인한다. 원본 검증 환경은 Windows 11, `C:\isaacsim-6.2`, `6.2.0-rc.14`, RTX 3070 8 GB였다. 대상 버전이 다르면 설치본 예제와 해당 버전 공식 문서로 호환성을 판단한다. SDK 실행에는 Isaac Sim의 런처를 사용한다.
- PowerShell에서 `$sim`을 실제 Isaac Sim 설치 폴더로 지정하고 저장소 루트에서 아래 명령을 실행한다. 공장 데모와 Panda 실습은 `/clock`을 각각 발행하므로 순서대로 실행한다.

### 2. 공장 라인 데모 실행

```powershell
# 이 PC에서 확인한 설치 경로로 바꾼다.
$sim = 'C:\isaacsim-6.2'
& "$sim\python.bat" rapid_style.py
& "$sim\python.bat" -m unittest discover -s tests -v
& "$sim\python.bat" factory_line_demo.py --headless --seconds 40
```

- 기대 결과는 `SELF-TEST PASS`(7항목), 단위 시험 6개 `OK`, `CHECK PASS` 14개와 `SELF-CHECK PASS`다. 40초는 시뮬레이션 시간이다. 첫 실행은 NVIDIA USD 에셋과 H1 보행 정책을 받느라 원본 PC에서 약 12분 걸렸다. 다운로드가 진행 중이면 기다린다. 접근 오류가 확인되면 Kit 로그의 실패 경로를 찾아 허용된 로컬 에셋 팩이 있는지 확인한다. 필요한 에셋을 확보하지 못하면 동일 재현이 막힌 이유로 보고한다.
- `/isaac_sim`은 `/rapid_controller` 자식 프로세스를 자동 실행한다. 두 프로세스는 번들 Jazzy와 Fast DDS(`rmw_fastrtps_cpp`)를 쓴다. 기본 실행의 검증에는 `--no-controller`를 빼고 `--motion` 없이 실행한다. 지령이 도착하지 않으면 두 노드의 시작 로그와 `/clock`, RMW, `ROS_DOMAIN_ID`부터 확인한다. 콘솔 종료 코드와 `SELF-CHECK` 결과를 모두 확인한다.
- `out/shot_06s.png`, `shot_16s.png`, `shot_26s.png`, `shot_36s.png`가 새로 저장됐는지 확인하고 연다. 박스 검사·적재, AMMR 왕복, H1 보행이 기대한 장면과 맞는지 본다. 박스는 시각적으로 그리퍼를 따라간다.
- `& "$sim\python.bat" factory_line_demo.py`로 GUI를 띄운다. KUKA KR210, Kawasaki RS080N, Ridgeback+Franka, Unitree H1의 동작을 확인한다. 팔레트를 클릭해 터미널의 `[TEACH] robtarget(...)` 출력을 확인한다. 정지(■) 후 재생(▶)해 H1이 다시 서서 걷는지 본다. 화면을 캡처하고 MoveIt 실습 전에 이 데모를 종료한다.

### 3. Panda MoveIt 실습 실행

1. `wsl -l -v`로 WSL2 배포판을 찾는다. 원본은 Ubuntu 24.04와 `/opt/ros/jazzy`를 썼다. Jazzy 셸에서 `ros2 pkg prefix`로 `moveit_configs_utils`와 `moveit_resources_panda_moveit_config` 등을 확인한다. 누락된 경우 `moveit_lab/README.md`의 패키지 목록을 따른다. Isaac Sim 측은 번들 Jazzy를 사용한다.
2. Windows에서 `& "$sim\python.bat" moveit_lab\isaac_panda_sim.py`를 띄우고 `[MOVEIT-LAB] /isaac_panda ready`를 기다린다. 다른 PowerShell에서 같은 `$sim`을 지정해 `& "$sim\python.bat" moveit_lab\send_trajectory.py`를 실행한다. `/panda_arm_controller/follow_joint_trajectory`로 보낸 궤적 2개가 성공하고 종료 코드가 0인지 확인한다.
3. WSL 터미널에서 아래 명령으로 `move_group`과 RViz를 띄운다. `repo`에는 대상 PC의 **Windows 저장소 폴더**를 `wslpath`로 변환한 경로를 넣는다. 두 ROS 2 환경의 `ROS_DOMAIN_ID`가 같아야 한다.

   ```bash
   source /opt/ros/jazzy/setup.bash
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   repo="$(wslpath 'D:\Projects\IsaacSimProcess')"
   ros2 launch "$repo/moveit_lab/wsl/moveit_isaac.launch.py"
   ```

4. 두 번째 WSL 터미널에서도 같은 `source`, `export`, `repo` 설정을 적용한다. `ros2 topic list`와 `ros2 action list`로 `/clock`, `/joint_states`, `/move_action`, `/panda_arm_controller/follow_joint_trajectory`가 보이는지 확인한다. 목록에 없다면 Isaac Sim 재생 상태, 두 환경의 RMW와 도메인, WSL 네트워크 및 Windows 방화벽을 진단한다. 설치본과 [NVIDIA ROS 2 문서](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_ros.html)의 해당 구성을 참고한다.

   ```bash
   cd "$repo/moveit_lab/wsl/examples"
   python3 ex1_named_pose.py
   python3 ex2_tcp_goal.py
   python3 ex3_obstacle.py
   python3 ex4_lin_vs_ompl.py
   ```

5. 네 예제의 `EXAMPLE PASS`를 확인한다. 관절 목표, TCP 목표(OMPL·PTP·LIN), 기둥 회피, 막힌 직선 경로의 거부와 우회가 실제로 실행되는지 본다. Isaac Sim의 Panda와 RViz의 자세가 함께 움직이는 화면을 캡처한다. 선택적으로 `ex3_obstacle.py --no-scene`의 의도된 `CONTROL_FAILED`를 비교한다. 충돌로 자세가 어긋났으면 Isaac Sim을 재시작하고 `/clock` 초기화에 맞춰 MoveIt launch도 다시 띄운다.

### 4. 결과 보고

실제로 사용한 설치 경로, Isaac Sim 버전, 실행 명령, 테스트 결과, 로그·캡처 경로를 전달한다. 공장 라인은 박스 적재 수, TCP 오차, H1 최저 골반 높이와 정지 후 재생 결과를 남긴다. MoveIt은 궤적 2개와 예제 4개의 결과를 각각 남긴다. 실패 시 첫 오류 원문과 취한 조치를 기록하고, 미실행·미확인 항목을 명시한다. 원인이 확인되면 해당 경로와 실행 환경만 최소 범위로 조정한 뒤 실패한 단계부터 다시 검증한다.
