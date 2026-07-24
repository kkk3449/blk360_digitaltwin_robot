# blk360_digitaltwin_robot

**BLK360 스캔 기반 IsaacSim 씬 ↔ 실물 AMR(ammr, 스워브 2모듈) 디지털 트윈 연동**

실물 로봇(ammr)의 자율주행 스택(Nav2 + AMCL)이 추정한 map 프레임 pose와 속도를
IsaacSim으로 실시간(30 Hz) 스트리밍하고, 역방향으로 IsaacSim에서 발행한 목표 위치로
실물 로봇을 주행시키는 양방향 디지털 트윈.

- 로봇: ROS2 Jazzy, CycloneDDS, `ROS_DOMAIN_ID=56`, 도커 컨테이너(`ammr-sj-dev`) 내부 구동
- IsaacSim PC: BLK360 실측 스캔 기반 씬, ROS2 Bridge(ActionGraph)
- **설계 원칙: 기존 자율주행 코드는 한 줄도 수정하지 않는다.** 별도 노드 + 별도 DDS 설정만 추가
  (예외: 라이다 끊김 근본 원인이던 DDS Peer 설정 1건 수정 — 아래 [4장](#4-라이다-주기적-끊김-해결-cyclonedds-peer-설정-변경) 참조)

---

## 1. 저장소 구성

```
digital_twin_bridge/        ROS2 패키지 (ament_python) — 트윈 브릿지 본체
├── digital_twin_bridge/
│   ├── state_publisher.py  dt_state_publisher: 로봇 → IsaacSim 상태 송신 (30Hz)
│   └── goal_relay.py       dt_goal_relay: IsaacSim → 로봇 목표 릴레이
├── launch/digital_twin.launch.py
├── config/cyclonedds_twin.xml   브릿지 전용 DDS 설정 (기존 스택 무침습)
└── README.md               상세 설명 (네트워크, IsaacSim 세팅 STEP 체크리스트 포함)

docs/
└── cyclonedds_robot.xml    로봇 본체(/root/cyclonedds.xml)의 최종 설정 (라이다 끊김 수정 반영본)
```

## 2. 데이터 흐름

| 방향 | 토픽 | 타입 | 내용 |
|---|---|---|---|
| ammr → IsaacSim | `/ammr/state` | `nav_msgs/Odometry` | map 프레임 pose + 선속도/각속도, 30 Hz |
| IsaacSim → ammr | `/ammr/goal_pose` | `geometry_msgs/PoseStamped` | 목표 위치 → 내부 `/goal_pose`로 릴레이 → Nav2 주행 |

pose 출처는 1순위 TF `map→base_footprint`(주행 중 연속 갱신), 2순위 `/amcl_pose`
(TRANSIENT_LOCAL QoS, 정지 시 latched 값). 속도는 `/odom`의 twist를 사용한다.

## 3. 실행 (로봇 쪽)

```bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select digital_twin_bridge
source install/setup.bash

# 자율주행 스택 기동 후 초기 포즈 지정 (AMCL 수렴에 필요)
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]}}"

# 트윈 브릿지 실행
ros2 launch digital_twin_bridge digital_twin.launch.py
```

브릿지 프로세스에만 `config/cyclonedds_twin.xml`이 주입되어(launch `additional_env`)
내부망(192.168.10.66)과 wifi(192.168.31.56) 양쪽 인터페이스에 바인딩된다.
IsaacSim PC 쪽 세팅과 트러블슈팅은 [`digital_twin_bridge/README.md`](digital_twin_bridge/README.md)의
STEP 체크리스트 참조.

---

## 4. 라이다 주기적 끊김 해결: CycloneDDS Peer 설정 변경

트윈 실험 중 "IsaacSim에서 pose가 멈춘다"는 증상의 근본 원인을 추적해 해결한 기록.
**비-DDS 장비를 CycloneDDS Peer로 등록하면 안 된다**는 것이 핵심 교훈이다.

### 증상
- SICK nanoScan3 라이다 2대(전방 192.168.10.88 / 후방 192.168.10.99)가 케이블 접촉과 무관하게 **주기적으로 데이터 중단** (ping도 함께 플래핑)
- 라이다가 죽으면 `/scan` 소실 → AMCL pose 갱신 중단 → 브릿지가 latched `/amcl_pose`로 폴백 → **IsaacSim에서 pose가 마지막 값으로 동결** (twist는 계속 살아있어 원인 파악이 어려웠음)
- 핑 정상 + CoLa2(TCP 2122) 제어 명령은 전부 ACK인데 UDP 측정 데이터만 0인 "반쯤 죽은" 상태도 발생 — 이 상태는 드라이버 재시작으로 안 풀리고 **라이다 전원 리셋으로만 복구**

### 원인
로봇의 `/root/cyclonedds.xml` Discovery Peers에 **DDS 장비가 아닌** 라이다 2대와
Beckhoff PLC(192.168.10.151)의 IP가 등록되어 있었다.

CycloneDDS는 Peer로 등록된 주소에 SPDP 유니캐스트 디스커버리 패킷을 지속 전송하는데,
로봇에서 도는 **모든 ROS 노드**가 각자 이 패킷을 쏘다 보니 실측 **초당 약 140패킷**(12초에
1,722패킷)이 라이다로 날아갔다. nanoScan3의 임베디드 네트워크 스택이 이 부하로 주기적으로
다운된 것이 끊김의 근본 원인이었다.

### 수정 (Before → After)

```xml
<!-- Before: /root/cyclonedds.xml -->
<Peers>
  ...
  <Peer address="192.168.10.88"/>   <!-- 전방 라이다 (삭제) -->
  <Peer address="192.168.10.99"/>   <!-- 후방 라이다 (삭제) -->
  <Peer address="192.168.10.151"/>  <!-- Beckhoff PLC (삭제) -->
</Peers>
```

```xml
<!-- After: DDS 노드가 실제로 도는 호스트만 Peer로 유지 -->
<Peers>
  <Peer address="192.168.31.25"/>
  <Peer address="192.168.31.56"/>
  <Peer address="192.168.31.103"/>
  <Peer address="192.168.31.77"/>
  <Peer address="192.168.31.234"/>
  <!-- 라이다(.10.88/.10.99)와 Beckhoff(.10.151)는 DDS 장비가 아님 - Peer 등록 금지.
       (등록 시 전체 ROS 노드가 디스커버리 패킷을 초당 ~140개 쏴서
        라이다 네트워크 스택이 주기적으로 다운됨. 2026-07-14 확인) -->
</Peers>
```

전체 최종 설정: [`docs/cyclonedds_robot.xml`](docs/cyclonedds_robot.xml)

### 검증
- 수정 후 라이다로 향하는 디스커버리 패킷 0개 확인 (tcpdump)
- 45초 연속 주행 테스트: `/scan` 33–34 Hz 유지, 라이다 끊김 0회, AMCL pose 연속 추적(약 3.6 m 왕복), `/ammr/state` 30 Hz 정상

### 부수적으로 확인된 운영 노하우
- `sick_safetyscanners2` 드라이버는 lifecycle cleanup 시 UDP 소켓(2111/2112)을 릭한다.
  같은 프로세스에서 재-configure 하면 `bind: Address already in use` →
  `host_udp_port` 파라미터를 임시로 다른 포트(2113/2114)로 바꾸고 configure 하면 우회 가능.
- `ros2 topic hz/echo`는 데이터가 없으면 SIGTERM을 무시하고 행업한다.
  스크립트에서는 `timeout -s KILL N`으로 감쌀 것.

---

## 5. 좌표계 정렬

IsaacSim 씬(BLK360 스캔)의 원점/축과 ammr `map` 원점(`origin: [-7.577, -10.929, 0]`,
resolution 0.05 m/px)이 일치해야 트윈이 겹친다. IsaacSim 월드 원점을 map 원점에 맞추거나,
ActionGraph에서 `/ammr/state`에 정적 offset을 적용한다.
