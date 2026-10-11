# crazyflie_mapf

이 폴더에서 기체, 목적지, 계획기, 시뮬레이터를 설정하고 실행하는 독립 ROS 2 패키지다.
7대 DMPC 교차 이동을 **장애물 없음 / 정적 장애물 / 동적 장애물** 모드로 실행한다.
PBS 핵심 `priority.py`와 `priority_demo`도 유지하지만, DMPC 비행에는 사용하지 않는다.

## 폴더 구성

```text
crazyflie_mapf/
├── config/
│   ├── robots.yaml           # 사용 기체, 연결 주소, 초기 위치
│   ├── goals.yaml            # 기체별 목적지
│   ├── obstacles_none.yaml   # 장애물 없음
│   ├── obstacles_static.yaml # 고정 박스의 위치·크기
│   ├── obstacles_dynamic.yaml # 움직이는 박스의 왕복 경로·주기
│   ├── planner.yaml          # 속도, 가속도, 안전거리, 예측 시간
│   ├── simulation.yaml       # SIL 제어기, 화면, 파일 경로
│   ├── visualization.rviz    # RViz 표시와 카메라
│   └── world.sdf             # 바닥·조명 (장애물은 YAML에서 생성)
├── launch/
│   ├── no_obstacles.launch.py
│   ├── static_obstacles.launch.py
│   ├── dynamic_obstacles.launch.py
│   └── simulation.launch.py  # 기존 명령 호환: 정적 모드
├── crazyflie_mapf/
│   ├── priority.py           # PBS 우선순위 탐색
│   ├── demo.py               # PBS 합성 조건 예제
│   ├── trajectory/           # DMPC QP, 검증, 지표, 시각화
│   ├── executor/             # 비행 실행기와 Gazebo pose 연동
│   ├── fleet/                # 목적지 배정과 완료 상태
│   └── common/               # simulation_launch.py: 공통 실행, 설정·ROS 도구
└── test/
```

실행 시 `crazyflie_test`와 `crazyflie_delivery`의 코드·설정·launch를 사용하지 않는다.
공통 기반인 ROS 2 Jazzy, Crazyswarm2의 `crazyflie_sim`, `crazyflie_py`,
`crazyflie_description`, firmware Python 바인딩(`cffirmware`), Gazebo Sim은 필요하다.
Python 의존성은 NumPy, SciPy, PyYAML, OSQP 1.x이며 기존 basic 이미지에 설치되어 있다.
원본 코드와 이 패키지의 후속 변경은 자동 동기화되지 않는다.

## 설정

- **robots.yaml**: `enabled`, `uri`, `initial_position: [x, y, z]`, `type`을 기체별로 지정한다.
  좌표는 `all.reference_frame` 기준이며 단위는 m다. Crazyradio는 IP 대신
  `radio://동글번호/채널/전송속도/기체주소`를 사용한다. 기본 주소는 예시다.
  이전 `crazyflies.yaml`은 `robots.yaml`로 이름을 변경했다.
- **goals.yaml**: `frame: world`와 `goals: {cf1: [x, y, z], ...}`로 목적지를 지정한다.
  초기 위치에서 자동 계산하지 않는다. 비활성 기체의 목적지는 무시한다.
- **obstacles_none/static/dynamic.yaml**: 각 launch가 해당 모드의 파일만 읽는다.
  기존 `obstacles.yaml`은 `obstacles_static.yaml`로 이름을 변경했다.
- **planner.yaml**: `speed`, `acceleration`, `deceleration`, `horizontal_separation`,
  `vertical_separation`, `horizon`, `plan_rate`, `takeoff_height`, `bounds` 등을 지정한다.
  현재 속도 상한은 2.5m/s, 가속도는 0.65m/s², 감속도는 0.9m/s²다.
  실제 속도는 경로 길이와 회피 제약에 따라 상한보다 낮을 수 있다.
- **simulation.yaml**: `operation`, `gazebo`, `gazebo_gui`, `rviz`, `render_engine`,
  `software_rendering`, `sim.controller` 등을 지정한다. 상대 파일 경로는 config 폴더 기준이다.
  `gazebo_ip: 127.0.0.1`은 같은 PC의 Gazebo 프로세스 간 통신 주소다.

기체 추가 시 robots와 goals에 같은 이름을 추가한다. 2대/4대로 제한한 코드는 없지만
현재 기본 설정은 7대이며 계산 비용은 기체 수에 따라 증가한다.
목적지 누락, 좌표계 불일치, 경계 밖/겹치는 목적지, 겹치는 이륙 위치는 시작 전에 거부한다.
`sim.visualizations.rviz.enabled`는 TF 위치 피드백 때문에 true로 유지한다.
`firmware_bindings`는 기존 Docker의 `/workspace/crazyflie-firmware/build`를 가리킨다.
네이티브 실행 시 바꾸거나, 이미 Python import가 가능하면 빈 문자열로 둔다.
이 launch는 항상 SIL 서버를 사용하며 무선 기체에 연결하지 않는다.

## 장애물 설정

`config/obstacles_static.yaml` 또는 `config/obstacles_dynamic.yaml`을 수정하고 launch를 재시작한다. 위치·크기의 단위는 m이며,
`position`은 **바닥 위치가 아니라 박스 중심**이다. 회전 없는 박스만 지원한다.

```yaml
frame: world
obstacles:
  - name: center_box
    enabled: true
    type: box
    position: [0.0, 0.0, 0.8]
    size: [0.5, 0.5, 1.6]
```

이 예제는 지면 위 높이 1.6m 박스다. 장애물을 더 만들려면 서로 다른 `name`으로
항목을 추가한다. `enabled: false`는 해당 박스를 끄며, `obstacles: []`는 모두 없앤다.
Gazebo의 박스와 RViz의 주황색 박스, DMPC 회피 계산은 같은 YAML을 읽는다.
RViz의 옅은 타원체는 기본 안전 영역이다.

계획기는 박스에 `robot_radius + collision_margin`을 더한 뒤 그 박스 전체를 감싸는
타원체를 사용한다. 각 반축은 `sqrt(3) * (size/2 + robot_radius + collision_margin)`이다.
추적·시간 불확실성도 계산 중 추가하므로 실제 박스보다 넓게 우회하며 좁은 통로는
사용하지 못할 수 있다. 연속 가속도 궤적 전체와 감속 fallback을 검사한다.
초기 이륙 경로와 목적지가 안전 영역에 들어가면 시작 전에 거부하고,
실행 중 요청하는 목적지도 검사한다. 잘못된 frame, 크기, 중복 이름은 오류로 처리한다.

이것은 정적/예측 가능한 동적 장애물에 대한 국소 DMPC 확장이다. 복잡한 미로에서 경로를 반드시
찾는 전역 계획기는 아니며, 해를 찾지 못하면 감속하거나 UNSAFE를 보고한다.
Gazebo 박스에는 collision 요소도 있지만 드론 동역학은 별도 SIL에서 계산되므로
Gazebo 접촉 물리가 비행을 막아주는 구조는 아니다.

## 세 가지 실행 모드

공통 기체·목적지·계획 설정은 그대로 공유하고 장애물 파일만 분리한다.
한 번에 하나의 launch만 실행한다. 이전 시뮬레이션은 먼저 종료한다.

| 실행 파일 | 사용하는 장애물 설정 |
| --- | --- |
| `no_obstacles.launch.py` | `config/obstacles_none.yaml` (`obstacles: []`) |
| `static_obstacles.launch.py` | `config/obstacles_static.yaml` |
| `dynamic_obstacles.launch.py` | `config/obstacles_dynamic.yaml` |

각 launch는 `crazyflie_mapf/common/simulation_launch.py`의 공통 실행 함수를 호출한다.
`simulation.launch.py`는 기존 사용자를 위해 정적 모드로 유지한다.
`config_dir`, `operation`, `gazebo`, `gazebo_gui`, `rviz` 인자는 세 모드 모두 동일하다.
장애물 없는 모드에 활성 박스를 넣거나 정적 모드에 `motion`을 넣으면 오류로 처리한다.

```bash
ros2 launch crazyflie_mapf no_obstacles.launch.py
ros2 launch crazyflie_mapf static_obstacles.launch.py
ros2 launch crazyflie_mapf dynamic_obstacles.launch.py
```

동적 모드의 기본 예제는 다음과 같다.

```yaml
frame: world
obstacles:
  - name: moving_box
    enabled: true
    type: box
    position: [0.0, -0.7, 0.8]
    size: [0.5, 0.5, 1.6]
    motion:
      type: ping_pong
      end_position: [0.0, 0.7, 0.8]
      period: 7.0
      start_time: 4.0
```

`position`에서 `end_position`까지 1.4m를 3.5초에 이동하고, 돌아오는 데 3.5초로 총 7초 주기다.
이동 속도는 0.4m/s다.
시뮬레이션 시작 후 4초까지 정지한 뒤 움직인다. 양 끝에서 즉시 방향을 바꾸는
등속 왕복 모델이며 회전은 없다. 동적 YAML 안에 `motion` 없는 정적 박스를 섞을 수 있다.

Gazebo pose, RViz, 계획기는 동일한 ROS `/clock` 시간과 이동 함수를 사용한다.
계획기는 예측 구간의 미래 장애물 위치를 사용하고, 방향 전환 시점에서 구간을 나눠
상대 운동의 연속 충돌을 검사한다. 감속 경로에도 같은 검사를 적용한다.
목적지에서 계속 호버하고 수직 이륙할 수 있도록, 목적지와 이륙 경로는 장애물의
**전체 왕복 범위를 감싼 보수적인 안전 영역** 밖에 있어야 한다.

이 모드는 미리 알려진 이동 경로를 예측하는 시뮬레이션이다. 센서로 감지한 임의의
이동 장애물을 추적하는 기능은 아니다. `operation:=preview`는 `/clock` 없는 ROS
시스템 시간을 사용하므로 왕복 위상이 SIL 실행과 다르며 대기 시간은 이미 지난 것으로 본다.

## 빌드·실행

호스트 `~/crazyflie`에서:

```bash
docker compose -f docker/compose.yml up -d basic
xhost +local:docker
docker compose -f docker/compose.yml exec basic bash
```

컨테이너 안에서:

```bash
cd /workspace/cf_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select crazyflie_mapf
source install/setup.bash
ros2 launch crazyflie_mapf simulation.launch.py
```

YAML은 symlink 설치되므로 값을 수정한 뒤 launch를 재시작하면 적용된다.
새 파일이나 실행 파일 구성을 추가하면 다시 빌드한다. 다른 ROS 시뮬레이션은 먼저
종료한다. `/fleet/*`, `/tf`, `/clock` 토픽을 사용한다. Ctrl+C, RViz 닫기 또는 Gazebo
종료 시 함께 시작한 프로세스를 종료한다. 모든 목적지에 도착하면 호버하며 자동 착륙하지 않는다.

```bash
# GUI 없는 SIL 검사
ros2 launch crazyflie_mapf simulation.launch.py gazebo:=false rviz:=false

# 비행 서버 없이 이상적 궤적만 표시 (Gazebo는 실행하지 않음)
ros2 launch crazyflie_mapf simulation.launch.py operation:=preview

# 패키지 소스의 설정 폴더를 명시
ros2 launch crazyflie_mapf simulation.launch.py config_dir:=/workspace/cf_ws/src/crazyflie_mapf/config

# PBS 예제 및 테스트
ros2 run crazyflie_mapf priority_demo
python3 -m pytest src/crazyflie_mapf/test -q
```

## 구현과 검증 범위

중앙 PC에서 기체별 DMPC 가속도 QP를 풀고 예측을 교환한다. 연속 구간 이격, 상태
신선도, 명령 만료, 감속 fallback을 검사한다. Gazebo는 SIL pose를 표시하는 시각화다.
정적 박스와 설정된 왕복 경로를 따르는 동적 박스의 회피를 지원한다. 박스 배송과 실기체 검증은 포함하지 않는다.
계획 갱신은 목표 5Hz이며 마감 시간을 보장하지 않는다. `/trajectory/status`의
`over_budget`과 계획 시간, `/fleet/status`의 도달 상태로 실제 진행을 확인한다.

계획기·실행기·편대·지표·시각화와 관련 회귀 테스트는 기존 교차 실험에서 가져왔다.
기존 드론 간 회피에 정적·동적 장애물 제약과 연속 충돌 검사를 추가했다.

참고 문헌은 PBS의 [Searching with Consistent Prioritization for Multi-Agent Path Finding](https://arxiv.org/abs/1812.06356)
및 DMPC 코드에 명시된 [arXiv:1909.05150](https://arxiv.org/abs/1909.05150)이다.
PBS의 `PrioritySearch.solve()`는 외부 `evaluate(candidate)` 충돌 검사기를 받는다.
데모의 합성 조건은 실제 충돌 검사기가 아니다. 논문 전체 재현이나 일반적인 최적성·완전성을 보장하지 않는다.

## 연구 문제 기록

[DMPC 문제 기록과 연구 기여 후보](docs/research_problem_log.md)에 속도 증가 실험에서 확인한
QP 수렴 실패, 회피 제약의 실행 불가능성, SIL 결과 차이와 감속 후 대응의 한계를 정리했다.
확인된 사실과 미검증 가설을 구분하며, 관련 로그와 분석 시점의 코드·설정 사본을 함께 보관한다.
해결책은 아직 운영 코드에 적용하지 않았다.

패키지 공개 전에는 package.xml의 maintainer 이메일을 실제 주소로 바꾼다.
