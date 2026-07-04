# 다음 작업: 실제 실행 전 안전/그리퍼 방향 보정

## 결론

Stage 3에서는 MoveIt plan-only가 성공했다. 하지만 실제 로봇을 움직이면 아직 위험하다.

이유는 다음과 같다.

1. 현재 trajectory의 각 joint 회전량을 검사하지 않는다.
2. JOINT6가 많이 회전하면 배선이 꼬이거나 끊길 수 있다.
3. 현재는 position-only planning이라 그리퍼 방향이 자유롭다.
4. 우리 그리퍼는 물체를 위/아래로 잡는 구조가 아니라 양옆에서 좁혀 잡는 구조다.
5. 현재 pregrasp는 z 방향 위쪽에 생성되므로 top-down 접근에 가깝다.

---

## TODO 1 - joint 회전량 계산기 추가

실제 execute 전에 planned trajectory 전체를 검사해야 한다.

각 joint마다 다음 값을 출력한다.

```text
현재 각도
trajectory 중 최소 각도
trajectory 중 최대 각도
마지막 각도
총 변화량 deg
```

출력 예시:

```text
JOINT1 current=10 deg, min=5 deg, max=40 deg, final=35 deg, delta=25 deg
JOINT2 ...
JOINT6 ...
```

초기 안전 기준은 보수적으로 잡는다.

```text
JOINT1 최대 변화량: 90 deg
JOINT2 최대 변화량: 90 deg
JOINT3 최대 변화량: 90 deg
JOINT4 최대 변화량: 90 deg
JOINT5 최대 변화량: 90 deg
JOINT6 최대 변화량: 90 deg 이하로 시작
```

이 값은 실제 배선 상태를 보고 더 줄여야 한다.

---

## TODO 2 - JOINT6 winding 제한

MoveIt/URDF에서 JOINT6가 continuous로 되어 있더라도 실제 로봇에서는 무한 회전하면 안 된다.

필요한 규칙:

```text
JOINT6가 허용 각도 이상 회전하면 trajectory reject
JOINT6는 가능한 짧은 방향으로 회전
한 바퀴 이상 도는 IK 해는 사용 금지
```

좋은 plan이라도 JOINT6가 많이 돌면 execute 금지다.

---

## TODO 3 - 그리퍼 방향 고정

현재 설정:

```text
use_position_only = true
```

이 설정은 TCP 위치만 맞추고 자세는 자유롭다.

실제 grasp에서는 다음처럼 바꿔야 한다.

```text
use_position_only = false
```

그리고 `gripper_tcp`의 orientation을 고정해야 한다.

목표:

```text
그리퍼가 물체의 양옆을 잡음
위/아래로 찍지 않음
그리퍼 closing 방향이 물체 폭 방향과 맞음
접근 방향은 수평 방향
JOINT6가 불필요하게 회전하지 않음
```

---

## TODO 4 - gripper_tcp 축 확인

RViz에서 TF를 켜고 `gripper_tcp` 좌표축을 확인해야 한다.

확인할 것:

```text
어느 축이 그리퍼가 앞으로 접근하는 방향인가
어느 축이 그리퍼가 닫히는 방향인가
어느 축이 위/아래 방향인가
```

이걸 확인한 뒤 side grasp orientation을 정해야 한다.

---

## TODO 5 - pregrasp 생성 방식을 z-offset에서 수평 offset으로 변경

현재 방식:

```text
grasp_target_pose = object_point + z_offset
pregrasp_pose = grasp_target_pose + z_extra
```

이 방식은 pregrasp가 target보다 위에 생긴다. 즉 위에서 아래로 접근하는 구조에 가깝다.

side grasp에서는 이렇게 바꿔야 한다.

```text
grasp_target_pose = 물체 옆면 또는 중심 근처의 side-grasp point
pregrasp_pose = grasp_target_pose - approach_direction * approach_distance
```

즉 pregrasp와 grasp의 높이는 거의 같고, x/y 방향으로 떨어져 있어야 한다.

예시:

```text
grasp_target:
  x = object_x
  y = object_y
  z = object_z + 안전 높이

pregrasp:
  x = object_x - approach_dx
  y = object_y - approach_dy
  z = grasp_target_z와 거의 같음
```

---

## TODO 6 - pregrasp -> grasp는 Cartesian path로 변경

현재 sequence는 두 번의 일반 MoveIt planning이다.

```text
current -> pregrasp
pregrasp -> grasp
```

앞으로는 다음 구조가 더 좋다.

```text
current -> pregrasp: 일반 planning
pregrasp -> grasp: 짧은 Cartesian path
```

이렇게 해야 파란 화살표 방향으로 직선 접근하는 grasp 동작을 만들 수 있다.

---

## TODO 7 - execute_enabled 안전장치 추가

실제 실행 코드는 기본값을 반드시 false로 둔다.

```text
execute_enabled:=false
```

실제 실행할 때만 명시적으로 true를 넣는다.

```text
execute_enabled:=true
```

execute 전에 반드시 통과해야 하는 조건:

```text
1. plan success
2. joint 회전량 검사 통과
3. JOINT6 winding 검사 통과
4. joint limit 검사 통과
5. collision 검사 통과
6. gripper 방향 검사 통과
```

---

## TODO 8 - 최종 실제 grasp sequence

나중에 실제 로봇에서는 다음 순서로 간다.

```text
1. 그리퍼 열기
2. current -> pregrasp planning
3. safety check
4. execute current -> pregrasp
5. pregrasp -> grasp Cartesian path
6. safety check
7. execute approach
8. 그리퍼 cm/mm 단위로 닫기
9. 살짝 들어올리기
10. place pose로 이동
11. 그리퍼 열기
12. home 복귀
```

---

## 다음에 바로 할 작업

다음 단계는 Stage 4-0이다.

```text
Stage 4-0: trajectory joint safety auditor
```

목표:

```text
계획 경로를 실행하지 않고,
각 joint가 얼마나 도는지 계산하고,
특히 JOINT6가 위험하게 도는지 검사한다.
```
