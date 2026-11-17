# 업무 PC 인계 프롬프트 — Isaac Sim 로봇 동작을 RAPID 방식으로 이어서 작업하기

> **쓰는 법 (사용자용).** 이 파일을 `factory_line_demo.py`, `rapid_style.py`와 같은 폴더에 둔다.
> 아래 1절의 빈칸을 채운 뒤 에이전트에게 "WORK_PC_PROMPT.md를 읽고 그대로 따라 줘"라고 한다.
> 2절부터는 에이전트가 읽는 지시다.

---

## 1. 이번에 할 일 (사용자가 채운다)

- 만들고 싶은 것. `[예: ○○ 라인에서 로봇 2대가 부품을 집어 지그에 놓는 동작]`
- 쓸 로봇과 기종. `[예: ABB IRB 2600 1대, AMR 1대]`
- 가진 3D 모델 파일과 형식. `[예: D:\models\cell.step, robot.urdf]`
- Isaac Sim 설치 위치와 버전. `[모르면 비워 둔다. 에이전트가 찾는다]`
- 인터넷으로 NVIDIA 에셋 서버에 접속할 수 있는가. `[예 / 아니오 / 모름]`
- 지켜야 할 제약. `[예: 회사 자료는 외부로 보내지 않는다, 정해진 폴더에서만 작업한다]`

빈칸이 있으면 에이전트는 작업을 시작하기 전에 그 항목만 사용자에게 묻는다.

---

## 2. 너의 역할

너는 이 폴더의 두 파일을 출발점으로 삼아, 1절에 적힌 업무 상황에 맞는 Isaac Sim 시뮬레이션을 만든다.
사용자는 ABB RobotStudio와 RAPID에 익숙한 로봇 엔지니어이고 Isaac Sim과 파이썬은 배우는 중이다.
코드를 새로 설계하지 말고 **두 파일이 이미 쓰는 방식을 그대로 이어 간다.** 방식의 내용은 4절에 있다.

두 파일은 사용자의 집 PC에서 만들고 검증한 것이다. 업무 PC는 환경이 다를 수 있다.
그래서 고치기 전에 3절의 순서로 "그대로 돌아가는지"부터 확인한다.

---

## 3. 두 파일과 처음에 확인할 것

### 파일 설명

| 파일 | 내용 |
| --- | --- |
| `rapid_style.py` | RAPID 방식의 데이터·명령·실행기. numpy와 scipy만 쓴다. Isaac Sim 없이 단독 시험이 된다 |
| `factory_line_demo.py` | RAPID 전용 Isaac Sim 6.2 standalone 예시. 창고에 컨베이어 라인을 만들고 로봇 4대(KUKA KR210, Kawasaki RS080N, Unitree H1, Ridgeback+Franka)를 움직인다 |

- 두 파일은 같은 폴더에 있어야 한다. `factory_line_demo.py`가 `rapid_style`을 import한다.
- 현재 전달할 버전은 저장소 HEAD다. `b82bdb4`는 시간표 코드가 포함된 과거 버전이므로 기준으로 되돌리지 않는다.
- 설명은 파일 안 주석에 자세히 있다. **이 문서를 읽은 다음 두 파일을 처음부터 끝까지 읽는다.**
  특히 `rapid_style.py` 맨 위 설명(RAPID 대응표, ABB와 다른 점)과 `factory_line_demo.py`의 구역 머리말을 본다.

`factory_line_demo.py`의 구역 순서는 아래와 같다.

1. 라인 배치 (상수)
2. RAPID 프로그램과 박스 신호 처리
3. IK·보행 제어
4. 장면 구성
5. 로봇을 제어 대상으로 잡기
6. RAPID 방식의 목표·공구 등록
7. 실행 상태·클릭 티칭·물리 콜백
8. 메인 루프
9. 자체 점검
10. 종료

### 확인 순서

아래를 차례로 하고 결과를 사용자에게 보고한다. 하나라도 다르면 고치기 전에 원인부터 찾는다.

1. **Isaac Sim 찾기.** 설치 폴더와 그 안의 `VERSION` 파일을 읽는다. 집 PC는 `C:\isaacsim-6.2`, 버전 `6.2.0-rc.14`였다.
2. **명령 층 단독 시험.** Isaac Sim을 띄우지 않으므로 몇 초면 끝난다. `PASS` 7줄과 `SELF-TEST PASS`가 나와야 한다.

   ```bash
   <Isaac Sim 폴더>\python.bat rapid_style.py
   ```

3. **예시 자체 점검.** `CHECK PASS` 14줄과 `SELF-CHECK PASS`가 나와야 한다.

   ```bash
   <Isaac Sim 폴더>\python.bat factory_line_demo.py --headless --seconds 40
   ```

4. **박스 신호 회귀 시험.** 3개 시험이 통과해야 한다.

   ```bash
   <Isaac Sim 폴더>\python.bat -m unittest discover -s tests -v
   ```

5. **화면 확인.** 자체 점검이 `out/` 폴더에 PNG 4장을 남긴다. 그 이미지를 직접 열어 로봇과 박스가 보이는지 확인한다.

주의할 점.

- 3번의 **첫 실행은 오래 걸린다.** 창고와 로봇 에셋 약 1 GB를 NVIDIA 서버에서 받는다. 집 PC에서 약 12분이었다.
  백그라운드로 돌리고 제한 시간을 길게 잡는다. 두 번째부터는 캐시를 써서 약 30초 만에 뜬다.
- 회사망에서 에셋 서버가 막혀 있으면 3번이 실패한다. 그때는 6절을 따른다.
- 집 PC에서 잰 값은 아래와 같다. 값이 크게 다르면 환경 차이를 의심한다.

| 항목 | RAPID 방식 |
| --- | --- |
| KUKA 손끝 오차 평균 / 최대 | 0.0056 / 0.0187 m |
| Kawasaki 손끝 오차 평균 / 최대 | 0.0129 / 0.0471 m |
| AMMR 베이스 최대 지연 | 0.103 m |
| H1 골반 최저 높이 | 0.871 m |
| 실시간 대비 속도 (RTX 3070 8 GB) | 0.72배 |

---

## 4. 이어 가야 할 방식

### 4.1 로봇 동작은 RAPID 방식으로 적는다

사용자는 "목표 좌표 + 속도 + zone + tool"로 명령하는 방식을 가장 잘 읽는다. 새 동작은 이 방식으로 먼저 적는다.

```python
tGripper = tooldata(0.0, 0.0, 0.02, 0.0, 0.0, 0.0)                 # 손끝 링크에서 본 TCP. [m] 위치, [deg] 방향
pPick = robtarget(1.7, 0.0, 1.12, 180.0, 0.0, 0.0)                 # [m] 위치, [deg] 방향. rx = 180이 공구가 바닥을 보는 자세


def pick_program():
    while True:                                                     # WHILE TRUE DO ... ENDWHILE
        yield MoveL(Offs(pPick, 0.0, 0.0, 0.45), v1500, z50, tGripper)   # yield 한 줄이 RAPID 명령 한 줄이다
        yield MoveL(pPick, v1000, fine, tGripper)
        yield WaitUntil(box_at_pick)                                # 센서 입력을 기다린다
        yield SetDO("doGrip", 1)
        yield WaitTime(0.3)
```

- 쓸 수 있는 것은 `robtarget`, `tooldata`, `jointtarget`, `basetarget`, `Offs`, `MoveL`, `MoveAbsJ`, `MoveBase`, `WaitTime`, `WaitUntil`, `SetDO`, 속도 `v100`~`v2000`, zone `fine`·`z1`~`z500`이다.
- 이동 로봇(AMR, 휴머노이드)도 같은 모양으로 적는다. `MoveBase(basetarget(x, y, rz), v500, fine)`.
- 새 명령이 필요하면 `rapid_style.py`에 **같은 모양(목표, 속도, zone)** 으로 추가하고 그 파일 아래의 단독 시험에 항목을 더한다.
- 이름은 RAPID 관례를 따른다. 목표는 `p…`, 공구는 `t…`, 관절 목표는 `j…`, 몸체 목표는 `b…`, 신호는 `do…`·`di…`.
- 프로그램 함수의 설명에는 같은 내용을 RAPID 문법으로 쓴 것을 나란히 적는다 (`palletize_program` 참고).
- ABB와 다른 점은 숨기지 않고 매번 밝힌다. 단위가 m·m/s인 것, 방향이 오일러 ZYX [deg]인 것, `MoveJ`와 `wobj`가 없는 것,
  zone이 포물선 코너 경로가 아닌 것, `MoveBase`가 RAPID에 없는 명령인 것.
- 컨베이어 트래킹과 "시각만 맞춘 것"을 구분해서 말한다. 지금 예시는 트래킹이 아니다.

### 4.2 좌표는 찍어서 얻는다

사용자는 RobotStudio처럼 물체에 스냅을 걸고 클릭해 타깃을 등록하는 방식을 원한다. 숫자를 실측해 대신 적어 주지 않는다.

1. 실행 중에 물체를 클릭하면 터미널에 `[TEACH] robtarget(...)` 한 줄이 찍힌다. 그 줄을 코드에 붙인다.
2. 코드에서는 `snap_top("/World/물체경로")`로 물체 윗면 가운데를 받는다.
3. 물체가 아니라 라인에서 정한 위치일 때만 숫자를 직접 적는다.

찍히는 점은 바운딩 박스의 윗면 중심 하나뿐이다. 다른 스냅이 필요하면 먼저 사용자와 방법을 정한다.

### 4.3 주석 규칙

- 새 소스 파일의 첫 줄은 그 파일의 역할을 적은 한국어 주석 한 줄이다.
- 숫자 상수는 **그 줄 주석의 맨 앞에 단위**를 적고 뒤에 설명을 쓴다. 예. `TAKT = 8.0  # [s] 택트 타임. 박스 1개가 나오는 간격`.
  좌표 쌍은 `[m] (x, y)`, 각도는 `[rad]`나 `[deg]`, 단위가 없으면 `[단위 없음]`.
- 구역마다 "무엇을 하는 부분인가"를 풀어서 적는다. 한 줄 요약으로 끝내지 않는다.
- 사용자가 자기 모델로 바꿔 쓸 지점에는 "내 로봇을 넣을 때는 무엇을 바꾸는가"를 적는다.
- 파이썬 고유 문법(`yield` 등)은 RAPID의 익숙한 개념(프로그램 포인터 등)에 빗대 설명한다.
- 직접 돌려 보지 않은 것은 주석에 그렇게 적는다.

### 4.4 RAPID 전용 구동을 유지한다

사용자의 2026-11-17 후속 요청으로 시간표 구동 코드와 모드 선택을 제거했다.
새 로봇도 RAPID 스타일 프로그램으로 작성한다. 로봇의 IK·관절 전송·보행 정책은 그 명령을 실행하는 필수 제어 계층이다.
이전 구현은 Git 이력에만 두며 현재 소스에 다시 추가하지 않는다.

### 4.5 검증 규칙

- 코드를 고쳤으면 자체 점검을 돌리고 결과 줄을 그대로 보여 준다. 통과하지 못했으면 그렇게 말한다.
- 화면으로 보이는 것만 믿지 않는다. 손끝 오차, 넘어짐 여부, 박스가 놓인 자리를 숫자로 검사한다.
- 자체 점검에는 **정지 후 재생 시험**이 들어 있다. 새 로봇을 넣어도 이 시험을 유지한다.
- 장면을 바꾸면 자체 점검 항목도 그 장면에 맞게 고친다. "목표를 따라가는가"와 "넘어지지 않는가"는 남긴다.
- 동작만 바꾸는 리팩터링은 바꾸기 전후의 출력이 같은지 수치로 비교한다.
- 직접 확인하지 못한 것은 "확인하지 않은 것"으로 따로 보고한다.

### 4.6 대화와 작업 방식

- 사용자에게 보내는 글은 한국어로 쓴다. 코드, 명령어, 경로, 오류 원문만 원문 그대로 둔다.
- 첫 줄에 결론을 한 문장으로 쓴다. 한국어 문장은 콜론으로 끝내지 않는다.
- 가정이 있으면 먼저 밝힌다. 해석이 여러 가지면 조용히 고르지 말고 보여 준다.
- 요청받은 것만 최소한의 코드로 만든다. 주변 코드를 덤으로 고치지 않는다.
- 오류가 나면 추측하지 말고 실제 오류 문장과 로그를 읽는다. 틀린 추정도 기록에 남긴다.
- 논리적으로 한 가지 변경이 끝나면 커밋한다. 커밋 메시지는 한 문장으로 쓴다.
- 작업이 여러 단계면 `checklist.md`(할 일)와 `context-notes.md`(결정과 이유)를 만들고 갱신한다.
- 회사 자료(3D 모델, 도면, 라인 정보, 제품명)를 외부 서비스로 보내지 않는다. 웹 검색어에도 넣지 않는다.

---

## 5. 이미 겪은 문제 (다시 겪지 않는다)

| 증상 | 원인 | 해결 (코드 위치) |
| --- | --- | --- |
| 팔이 이동 구간에서 목표보다 최대 1 m 뒤처진다 | IK 출력을 "측정 관절각 + 증분"으로 넣으면 관절 속도가 `kp/kd × 증분`에 묶인다 | 지령 관절각을 따로 적분한다 (`joint_step`) |
| 정지 후 다시 재생하면 휴머노이드가 넘어진다 | 정지하면 PhysX가 관절 게인을 에셋 기본값으로 되돌린다. 정책 러너는 학습 게인을 처음 한 번만 넣는다 | 정지를 감지하면 `h1.close()`를 부른다 (`frame`) |
| 특정 코드가 아니라 모든 호출이 고르게 느리다 | numpy·scipy·torch의 계산용 스레드가 물리·화면 스레드와 CPU를 다툰다 | `SimulationApp`을 만들기 전에 스레드 수를 1로 정한다 (파일 맨 위) |
| KUKA 관절이 지령 없이 흐른다 | 에셋 기본 게인(5.7e16)이 지나치게 크다 | 5.73e8 / 5.73e6으로 낮춘다 (`on_physics_step`의 첫 스텝) |
| 팔이 접히면서 목표를 못 따라간다 | 집는 점과 놓는 점을 잇는 직선이 로봇 축 가까이 지나간다 | 로봇 위치를 옮겨 경로가 축에서 1 m 안팎을 유지하게 한다 |
| 로봇이 도착하기 전에 그리퍼 신호가 나간다 | 지령이 목표에 닿아도 실제 로봇은 조금 늦게 온다 | `fine`은 실제 TCP가 5 mm 안에 들어올 때까지 기다린다 (`RapidTask._move_l`) |
| 새 에셋을 쓰는 첫 실행이 10분 넘게 걸린다 | 에셋을 서버에서 받는다 | 백그라운드로 돌리고 기다린다. 받은 것은 캐시에 남는다 |

그 밖에 알아 둘 것.

- Isaac Sim 6.2에서 `isaacsim.core.api`(World)는 `extsDeprecated`로 옮겨졌다. 이 예시는 `isaacsim.core.experimental.*`를 쓴다.
- Isaac Sim 안에서 `config` 같은 흔한 이름의 모듈을 만들면 번들된 패키지와 이름이 부딪힐 수 있다. 새 모듈 이름은 구체적으로 짓는다.
- 관절 부호 규약을 모르는 로봇은 자세를 손으로 맞추지 않는다. 손끝 목표를 월드 좌표로 주고 IK에 맡기는 편이 빨랐다.

---

## 6. 환경이 집 PC와 다를 때

- **Isaac Sim 버전이 다르다.** API 이름이 다를 수 있다. 기억으로 고치지 말고 그 설치본의 예제와 문서를 읽는다.
  `<Isaac Sim 폴더>\standalone_examples\api\isaacsim.core.experimental.api`,
  `<Isaac Sim 폴더>\standalone_examples\api\isaacsim.robot.policy.examples\h1_standalone.py`,
  `<Isaac Sim 폴더>\skills\SKILLS.md`가 출발점이다.
- **에셋 서버에 접속할 수 없다.** `get_assets_root_path()`가 `None`이면 예시가 시작하자마자 멈춘다.
  이 예시의 창고, 컨베이어, 팔레트, 로봇 3대, H1 모델과 H1 보행 정책이 모두 서버에서 온다.
  그때는 예시를 그대로 돌리려 하지 말고, 사용자의 모델 파일만으로 장면을 짜는 쪽으로 바로 간다.
  `rapid_style.py`와 IK, 실행 구조는 서버 없이도 쓸 수 있다. 사내에 에셋 팩이 있는지는 사용자에게 묻는다.
- **그래픽카드가 다르다.** 집 PC는 RTX 3070 8 GB였다. 물리는 CPU로 돌리고 그래픽 메모리는 화면에 쓴다.
- **폴더가 다르다.** 경로를 코드에 박아 넣지 않는다. 캡처는 스크립트 옆 `out/`에 생긴다.

---

## 7. 내 모델, 내 라인으로 바꾸는 순서

자세한 설명은 `factory_line_demo.py`의 "내 3D 모델 파일로 장면을 짤 때"와 "로봇을 제어 대상으로 잡기" 주석에 있다.

1. **모델 형식을 확인한다.** USD는 그대로 놓는다. STEP 같은 CAD, OBJ·FBX·glTF, URDF·MJCF는 먼저 USD로 바꾼다.
   소품은 모양만 있어도 놓을 수 있다. 로봇은 강체, 관절, 구동기, Articulation Root가 있어야 움직인다.
2. **로봇을 조사한다.** 아래 스크립트로 관절 이름, 한계, 링크 이름을 출력한다. 집 PC에서 실행해 확인한 코드다.

   ```python
   from isaacsim import SimulationApp

   app = SimulationApp({"headless": True})

   import isaacsim.core.experimental.utils.stage as stage_utils
   import omni.timeline
   from isaacsim.core.experimental.prims import Articulation
   from isaacsim.core.simulation_manager import SimulationManager

   SimulationManager.set_physics_sim_device("cpu")
   stage_utils.create_new_stage(template="sunlight")
   stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
   stage_utils.add_reference_to_stage("D:/models/my_robot.usd", "/World/robot")  # 내 로봇 USD의 전체 경로
   while stage_utils.is_stage_loading():
       app.update()

   omni.timeline.get_timeline_interface().play()
   for _ in range(5):
       app.update()

   robot = Articulation("/World/robot")
   print("DOF", robot.dof_names)
   lower, upper = robot.get_dof_limits()
   print("LIMITS", lower.numpy()[0], upper.numpy()[0])
   print("LINKS", robot.link_names)
   app.close()
   ```

   집 PC에서는 로봇 경로가 NVIDIA 에셋 서버 주소였다. 위처럼 로컬 파일 경로를 넣은 형태는 실행해 보지 않았다.
3. **공구 방향을 정한다.** 관절 하나에 +0.5 rad을 넣고 손끝 링크 좌표가 어느 쪽으로 가는지 본다.
   손끝 링크의 어느 축이 공구가 뻗는 방향인지 알아내 `tooldata`의 회전에 반영한다 (`tScanner` 참고).
4. **장면에 놓는다.** `add()`로 놓고 `make_arm()`에 경로, 손끝 링크, 공구 축, 시작 자세를 준다.
   시작 자세는 팔꿈치가 굽은 자세로 정한다. 팔이 곧게 펴진 자세는 IK가 불안정하다.
5. **목표를 찍는다.** 4.2의 방법으로 `robtarget`을 얻는다.
6. **프로그램을 쓴다.** 4.1의 모양으로 쓰고 물리 콜백에 태스크를 하나 더한다 (`state["tasks"]` 참고).
7. **자체 점검을 고친다.** 새 로봇의 손끝 오차 항목을 더하고 전체를 다시 돌린다.

한 번에 한 단계씩 하고, 단계마다 돌려 본 결과를 보여 준다.

---

## 8. 확인되지 않은 것

집 PC에서 직접 확인하지 못한 채로 넘어온 것이다. 필요해지면 먼저 확인한다.

- GUI에서 마우스로 물체를 클릭했을 때의 티칭. 같은 선택 이벤트를 코드로 일으켜서만 시험했다.
- RAPID 방식을 GUI 창으로 실행하는 것. 헤드리스 자체 점검과 캡처로만 확인했다.
- 일시정지 후 재생. 멈춘 곳에서 이어지도록 만들었지만 실행해 보지 않았다.
- 40초를 넘는 실행. 휴머노이드가 순찰 경로를 한 바퀴 완주하는 것은 보지 못했다.
- 프로그램 도중에 tool을 바꾸는 경우, `rz`가 0이 아닌 목표, 휴머노이드에게 `fine`으로 서 있으라고 하는 경우.
- 로컬 경로의 USD를 `add()`로 놓는 것, CAD Converter와 URDF 가져오기 도구를 쓰는 것.
- 로봇끼리의 충돌 회피는 없다. 경로가 겹치지 않게 배치했을 뿐이다.
- 박스는 물리가 없는 큐브다. 그리퍼가 실제로 쥐지 않고 박스가 TCP 위치를 따라간다.

---

## 9. 쓸 만한 스킬

- **Isaac Sim 설치본의 스킬.** `<Isaac Sim 폴더>\skills\SKILLS.md`에 목록이 있다. 필요한 것만 그때그때 읽는다.
  모델 변환은 `urdf-mjcf-to-usd-conversion`, 관절 구성은 `usd-articulation`, 에셋 접근 문제는 `isaac-sim-assets`,
  오류 진단은 `isaac-sim-troubleshooting`, 이동 경로 계획은 `motion-generation`과 `manipulation-ik`가 관련 있다.
- **에이전트 쪽 스킬.** 이 세션에 커밋, 테스트 실행, 인계 문서 작성 스킬이 있으면 해당 작업에서 쓴다. 없으면 직접 한다.

---

## 10. 끝났다고 말할 수 있는 조건

- 3절의 확인 결과를 보고했다 (그대로 돌아가는지, 다르면 무엇이 다른지).
- 1절의 동작이 RAPID 방식 프로그램으로 적혀 있고, 사용자가 좌표와 속도를 직접 고칠 수 있다.
- 자체 점검이 통과했고 그 출력을 보여 줬다. 캡처 이미지를 직접 열어 확인했다.
- 주석이 4.3의 규칙을 따른다.
- 확인하지 못한 것을 따로 적어 보고했다.
