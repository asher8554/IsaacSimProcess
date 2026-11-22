---
name: pitfalls
description: 실패에서 배운 함정. 같은 증상이 보이면 여기부터 본다
updated: 2026-11-22
---

## IK 지령은 따로 적분한다
"측정 관절각 + 증분"으로 넣으면 PD 구동의 정상 속도가 `kp/kd × 증분`(1 rad/s)에 묶여 이동 중 1 m 뒤처진다. 지령 관절각을 따로 적분한다(`joint_step`). 관절 속도 한계는 원인이 아니었다.

## 정지 후 재생하면 H1이 넘어진다
정지하면 PhysX가 관절 게인을 USD 기본값(kp 5730)으로 되돌린다. `RobotPolicyRunner`는 학습 게인을 런타임 생성 때만 넣는다. 정지를 감지하면 `h1.close()`. 정지 판별은 `timeline.is_stopped()`(일시정지와 구분).

## 수치 라이브러리 스레드
numpy·scipy·torch 계산 스레드가 물리·화면 스레드와 CPU를 다퉈 전체가 느려졌다. `OMP_NUM_THREADS` 등 3개를 1로(`setdefault`). 0.41x → 0.72x (2026-11-13). scipy 속도·GC는 원인이 아니었다.

## 빈 spin_once 비용
번들 rclpy `spin_once(timeout_sec=0)`는 할 일이 없어도 약 0.19 ms. 매 스텝 20번이면 4 ms. 처리한 메시지가 없으면 멈춘다 (2026-11-22).

## Kit의 Ctrl+C는 atexit을 건너뛴다
`SimulationApp`의 SIGINT 처리기가 `app.shutdown()`·`unload_all_plugins()` 중 프로세스를 끝낸다. 정리 작업을 atexit에 맡기지 않는다 (2026-11-22, `_thread.interrupt_main()`으로 재현).

출처: log/2026-11.md `2026-11-12 문제 해결 이력`, `2026-11-22 검토 수정`
