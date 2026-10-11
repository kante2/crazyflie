# crazyflie

Crazyflie 드론 개발 통합 저장소. Docker로 개발 환경을 통일한다.

## 다중 드론 DMPC 프로젝트

[`crazyflie_mapf`](cf_ws/src/crazyflie_mapf/README.md)는 7대 Crazyflie의 교차 이동과
정적·예측 가능한 동적 장애물 회피를 실험하는 독립 ROS 2 패키지다.
중앙 PC에서 기체별 가속도 QP를 풀고 예측 궤적을 교환한다.
목적지 관리, 궤적 계획, 비행 실행을 분리했으며 연속 시간 충돌 검사,
명령 만료 검사와 감속 fallback을 포함한다.

- **환경:** Python, ROS 2 Jazzy, NumPy/SciPy, OSQP, Crazyswarm2 SIL, Gazebo Sim, RViz2.
- **기본 설정:** 4초 예측 구간, 0.2초 간격, 목표 5Hz 재계획, 100Hz 명령 전송.
- **실험 모드:** 장애물 없음, 정적 박스, 정해진 경로로 왕복하는 동적 박스.
- **검증 자료:** [회귀 테스트](cf_ws/src/crazyflie_mapf/test/)와
  [실패 분석 및 연구 문제 기록](cf_ws/src/crazyflie_mapf/docs/research_problem_log.md).

Gazebo는 SIL 위치를 표시한다. 실기체 성능, 실시간 마감, 임의 환경에서의 도달과
충돌 회피를 보장하지 않는다. PBS 우선순위 탐색은 별도 라이브러리로 보관하며
현재 DMPC 비행에는 연결하지 않는다.

### 복제 및 실행

```bash
git clone --recursive https://github.com/kante2/crazyflie.git
cd crazyflie

# 새로 복제한 소스에 한 번 적용: 실험에서 사용한 SIL TF 발행 속도 제한.
git -C cf_ws/src/crazyflie-basic/crazyswarm2 apply \
  "$PWD/docker/patches/crazyswarm2-tf-rate.patch"

docker compose -f docker/compose.yml build basic
docker compose -f docker/compose.yml up -d basic
xhost +local:docker
docker compose -f docker/compose.yml exec basic bash
```

컨테이너 안에서:

```bash
cd /workspace/cf_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to crazyflie_mapf
source install/setup.bash

# 한 번에 하나의 모드만 실행한다.
ros2 launch crazyflie_mapf no_obstacles.launch.py
# ros2 launch crazyflie_mapf static_obstacles.launch.py
# ros2 launch crazyflie_mapf dynamic_obstacles.launch.py

# GUI 없는 SIL 실행
# ros2 launch crazyflie_mapf dynamic_obstacles.launch.py gazebo:=false rviz:=false

# 회귀 테스트
python3 -m pytest src/crazyflie_mapf/test -q
```

TF 패치는 `simulation.yaml`의 `sim.visualizations.rviz.max_rate`를 지원하며,
물리·제어 적분을 변경하지 않는다. 적용 대상 소스는 서브모듈 커밋으로 고정되어 있다.
상세 설정과 실행 옵션은 [패키지 README](cf_ws/src/crazyflie_mapf/README.md)를 참고한다.

## 구성

- [`crazyflie-firmware/`](crazyflie-firmware/) — 드론 펌웨어. [bitcraze/crazyflie-firmware](https://github.com/bitcraze/crazyflie-firmware) git submodule.
- [`cf_ws/src/crazyflie-basic/`](cf_ws/src/crazyflie-basic/) — 드론 제어용 ROS 2 패키지 (crazyswarm2 기반). [IN-AIR-KR/crazyflie-basic](https://github.com/IN-AIR-KR/crazyflie-basic) git submodule (`master` 추적).
- [`docker/`](docker/) — 실행 환경 + 편의 스크립트.
- [`cf_ws/src/crazyflie_mapf/`](cf_ws/src/crazyflie_mapf/) — 다중 드론 DMPC 궤적 계획과 장애물 회피 실험.

## 사전 준비

- Docker Engine + Docker Compose v2
- Linux, X11/XWayland (GUI용)
- (선택) GPU 쓸 경우 `nvidia-container-toolkit`
- Crazyradio PA 또는 Crazyflie USB 동글

## 실행

```bash
git clone --recursive https://github.com/kante2/crazyflie.git crazyflie   # 최초 1회
cd crazyflie

./docker/build.sh          # 이미지 빌드
./docker/container_start.sh  # 컨테이너 켜기
./docker/cf_basic.sh       # basic 컨테이너 접속
./docker/cfclient.sh       # cfclient(드론 GUI) 실행
```

GPU 쓰는 경우 `build.sh`/`container_start.sh` 대신:
```bash
docker compose -f docker/compose.yml -f docker/compose.gpu.yml build
docker compose -f docker/compose.yml -f docker/compose.gpu.yml up -d
```

**시뮬레이션 확인** (`cf_basic.sh`로 접속한 뒤, 최초 1회 빌드):
```bash
colcon build --symlink-install
```

빌드 후에는 컨테이너 밖에서 `launch.sh`로 바로 실행할 수 있다. `backend:=sim`이면 RViz2와 Gazebo Sim이 함께 열리고, RViz2 창을 닫으면 launch와 Gazebo도 같이 종료된다:
```bash
./docker/launch.sh                    # launch.py 기본값
./docker/launch.sh backend:=sim       # SIL + RViz2 + Gazebo Sim
./docker/launch.sh backend:=sim gazebo:=False  # Gazebo 없이 SIL + RViz2
./docker/launch.sh mode:=mocap backend:=cflib   # 실제 드론
```

Gazebo에는 `crazyflies_<mode>.yaml`에서 활성화된 드론이 생성되며, Crazyswarm2 SIL이 계산한 자세를 30 Hz로 반영한다. 따라서 현재 Gazebo 연동은 **비행 시각화 용도**이고, 충돌·모터·공기역학을 Gazebo가 계산하는 물리 시뮬레이션은 아니다.

> 이 환경은 ROS 2 Jazzy와 공식 조합인 **Gazebo Sim**을 사용한다. 인터넷 자료에서 흔히 보이는 회색 UI의 **Gazebo Classic(Gazebo 11)**과는 다른 후속 제품이다. Gazebo Classic용 플러그인이나 world 파일은 그대로 사용할 수 없다.

### 단일 기체 키보드 조종

`single_cf.py`는 소스에서 직접 실행되므로 텔레옵 스크립트 사용에는 재빌드가 필요 없다. `ros2 run crazyflie_test single_cf` 명령도 사용하려면 최초 한 번 패키지를 다시 빌드한다:

```bash
docker compose -f docker/compose.yml exec basic bash -c \
  "source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
   colcon build --symlink-install --packages-select crazyflie_test"
```

터미널 1에서 서버와 시각화를 실행하고, 터미널 2에서 키보드 조종기를 실행한다:

```bash
# 터미널 1: 시뮬레이션 + RViz2 + Gazebo Sim
./docker/launch.sh backend:=sim

# 터미널 2: 키보드 입력을 받는 단일 기체 조종기
./docker/single_cf.sh
```

시뮬레이션에서는 자동으로 `setpoint` 모드가 선택된다. 주요 키는 `t` 이륙, `l` 착륙, `w/a/s/d` 수평 이동, `q/e` 회전, `r/f` 상승·하강, `h` 정지, `Esc` 안전 종료이며 `x`는 즉시 모터를 끄는 비상 정지다. 옵션은 그대로 전달할 수 있다:

```bash
./docker/single_cf.sh --height 0.5 --speed 0.3
./docker/single_cf.sh --mode setpoint
```

**실제 드론 사용**:
1. Crazyradio를 꽂는다.
2. `./docker/cfclient.sh`에서 "Scan"으로 드론 연결 (필요하면 Bootloader 탭에서 펌웨어 플래시).
3. `cf_ws/src/crazyflie-basic/crazyflie_test/config/crazyflies_<mode>.yaml`의 `uri`를 내 드론 주소로 수정 후 `backend:=cflib`로 launch.

**끝내기**:
```bash
docker compose -f docker/compose.yml down
```

## 문제 해결

USB/GPU/X11/의존성 이슈는 [`docker/README.md`](docker/README.md) 참고.
