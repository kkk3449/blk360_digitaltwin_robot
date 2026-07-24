# digital_twin_bridge

ammr(실물 모바일 매니퓰레이터, 스워브 2모듈) ↔ IsaacSim PC 디지털 트윈 연동 브릿지.

기존 자율주행 코드는 **전혀 수정하지 않고**, 별도 노드/별도 DDS 설정으로만 동작한다.
(실행 중인 Nav2 스택에 무침습)

---

## 1. 데이터 흐름

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| **IsaacSim → ammr** | `/ammr/goal_pose` | `geometry_msgs/PoseStamped` | IsaacSim이 목표위치 발행 |
| (내부 릴레이) | `/goal_pose` | `geometry_msgs/PoseStamped` | `amr_interface`가 이미 구독 → NavigateToPose 주행 |
| **ammr → IsaacSim** | `/ammr/state` | `nav_msgs/Odometry` | map 프레임 pose(x,y,z,자세) + 선속도/각속도 |
| (편의용) | `/ammr/pose` | `geometry_msgs/PoseStamped` | map 프레임 자세만 |
| (편의용) | `/ammr/twist` | `geometry_msgs/TwistStamped` | 선속도/각속도만 |

- **pose 출처**: 1순위 TF `map→base_footprint`(주행 중 연속), 2순위 `/amcl_pose`(정지 시 latched 값).
- **속도 출처**: `/odom`(nav_msgs/Odometry)의 twist. 선속도 `twist.linear.x/y`, 각속도 `twist.angular.z`.

---

## 2. 노드 (ammr 도커 `ammr-sj-dev` 내부)

- `dt_state_publisher` : 상태 송신 (기본 30Hz)
- `dt_goal_relay`      : 목표 수신 릴레이

## 3. 빌드 & 실행 (ammr)

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select digital_twin_bridge
sds        # = source install/setup.bash  (새 노드 인식에 필수)

# 실행
ros2 launch digital_twin_bridge digital_twin.launch.py
# (선택) 별칭 등록:  alias twin='ros2 launch digital_twin_bridge digital_twin.launch.py'
```

실행 인자:
- `cyclonedds_uri:=<path>` — 브릿지 전용 DDS 설정 경로. 기본값은 이 패키지의
  `config/cyclonedds_twin.xml`. 기존 설정으로 되돌리려면 `:=/root/cyclonedds.xml`.
- `rate:=30.0`, `map_frame:=map`, `base_frame:=base_footprint`.

> DDS 설정은 **브릿지 프로세스에만** 주입된다(launch의 `additional_env`).
> 이미 도는 자율주행 노드들은 원래 `/root/cyclonedds.xml`을 그대로 쓴다.

---

## 4. 네트워크 (가장 중요)

현재 ammr의 `/root/cyclonedds.xml`은 `192.168.10.66`(eno1, 로봇 내부망)에 바인딩되어 있어
**그대로는 아이작심 PC(192.168.31.135)와 통신 불가**.

이 패키지의 `config/cyclonedds_twin.xml`은 브릿지 프로세스만
`192.168.10.66`(내부망) + `192.168.31.56`(ammr20_test wifi) **두 인터페이스에 바인딩**하고,
IsaacSim(`192.168.31.135`)을 Discovery Peer로 등록한다.

- ammr 무선 IP: `192.168.31.56` (enp44s0, ammr20_test)
- IsaacSim PC IP: `192.168.31.135`
- 공통: `ROS_DOMAIN_ID=56`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

---

## 5. IsaacSim PC 쪽 설정 (caselab@192.168.31.135)

IsaacSim의 ROS2 Bridge가 ammr과 같은 DDS 도메인/설정으로 붙어야 한다.

### 5-1. 환경변수 (IsaacSim 실행 셸)
```bash
export ROS_DOMAIN_ID=56
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/caselab/cyclonedds_isaac.xml
```

### 5-2. `~/cyclonedds_isaac.xml` (IsaacSim PC에 생성)
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface address="192.168.31.135" multicast="default"/>
      </Interfaces>
      <MaxMessageSize>64 kB</MaxMessageSize>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.31.56"/>   <!-- ammr -->
      </Peers>
      <LeaseDuration>30 s</LeaseDuration>
    </Discovery>
  </Domain>
</CycloneDDS>
```

> IsaacSim에 내장된 `omni.isaac.ros2_bridge`는 자체 rmw 라이브러리를 쓴다.
> CycloneDDS를 강제하려면 `FASTRTPS` 대신 CycloneDDS를 쓰도록
> `RMW_IMPLEMENTATION`을 IsaacSim 프로세스 환경에 설정해야 한다.
> (양쪽 rmw가 CycloneDDS로 동일해야 통신됨)

### 5-3. IsaacSim ActionGraph 구성
- **송신(→ammr, 목표)**: `ROS2 Publish PoseStamped` 노드 → 토픽 `/ammr/goal_pose` (frame_id=`map`).
- **수신(←ammr, 상태)**: `ROS2 Subscribe Odometry` 노드 → 토픽 `/ammr/state` →
  로봇 프림(prim) pose/속도에 반영 → 트윈 시각화.
- 두 노드의 `ROS2 Context` domain_id = **56** 확인.

---

## 6. IsaacSim PC 세팅 STEP 체크리스트 (caselab@192.168.31.135)

> ⚠️ 핵심 원칙: 브릿지 프로세스는 CycloneDDS여야 로컬 자율주행 스택(같은 호스트)을 본다.
> 한 프로세스는 rmw 하나만 쓴다 → **IsaacSim도 반드시 CycloneDDS.** (FastDDS↔CycloneDDS 혼용 공식 미지원)

**STEP 0 — 연결 확인**
```bash
ping 192.168.31.56          # ammr, 성공해야 함
```

**STEP 1 — `~/cyclonedds_isaac.xml` 생성** (위 5-2 내용 그대로)

**STEP 2 — IsaacSim 실행 셸 환경변수 (실행 *전*)**
```bash
export ROS_DOMAIN_ID=56
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/caselab/cyclonedds_isaac.xml
# IsaacSim이 시스템 ROS2를 쓰는 구성이면 먼저: source /opt/ros/<distro>/setup.bash (Jazzy 권장, ammr과 동일)
```
→ **이 셸에서 IsaacSim 실행** 후 ROS2 Bridge 확장(`isaacsim.ros2.bridge` 또는 구버전 `omni.isaac.ros2_bridge`) 활성화.

**STEP 3 — ⭐ DDS 핸드셰이크 게이트 (ActionGraph 만들기 *전에* 반드시 통과)**
ammr에서 `ros2 launch digital_twin_bridge digital_twin.launch.py` 실행 후,
IsaacSim PC의 **같은 환경 터미널**에서:
```bash
ros2 topic list | grep ammr     # /ammr/state, /ammr/pose, /ammr/goal_pose 보여야 함
ros2 topic echo /ammr/state     # pose + 속도 스트림 나와야 함
ros2 topic hz /ammr/state       # ~30Hz 확인
```
**여기까지 되면 네트워크/DDS는 100% 해결.** 안 되면 ActionGraph 만들어도 소용없으니 여기서 잡을 것.

**STEP 4 — ActionGraph 구성** (위 5-3)

**STEP 5 — 좌표계 정렬** (아래 7장)

**STEP 6 — 목표 주입 end-to-end 테스트** (⚠️ 실제 로봇이 움직임)
```bash
ros2 topic pub --once /ammr/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

### 6-1. 트러블슈팅 (STEP 3에서 막힐 때, 우선순위순)
1. **ammr 토픽이 아예 안 뜸** → rmw 불일치. IsaacSim이 진짜 CycloneDDS로 떴는지 확인
   (`echo $RMW_IMPLEMENTATION`, IsaacSim 로그의 rmw 표기). 내장 ROS2에 cyclonedds 없으면 시스템 ROS2 소싱.
2. **토픽 이름은 뜨는데 `echo`가 빔** → QoS/멀티캐스트 문제. wifi 멀티캐스트 차단 시 xml의 unicast Peer로 해결(이미 반영됨).
3. **방화벽** → IsaacSim PC: `sudo ufw allow from 192.168.31.0/24` (또는 임시 `sudo ufw disable`).
4. **도메인** → 양쪽 `echo $ROS_DOMAIN_ID` 모두 56 인지.

---

## 7. 좌표계 정렬(디지털 트윈 필수 확인사항)

IsaacSim 씬(BLK360 스캔 기반)의 **원점/축**과 ammr `map`(`geumjeong_260316`)의 원점이
일치해야 트윈이 겹친다.

- ammr map 원점: `origin: [-7.577, -10.929, 0]`, `resolution: 0.05 m/px`
- IsaacSim 월드 원점을 ammr `map` 프레임 원점과 동일하게 맞추거나,
  `/ammr/state`를 IsaacSim 월드좌표로 변환하는 정적 offset(TF)을 IsaacSim ActionGraph에 둘 것.
