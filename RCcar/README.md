# RC Car Project Overview

## 1. RC카 시스템 구조 (한 눈에 보기)

ROS 기반 SLAM/Planner가 만들어낸 **주행/지도 결과**를 받아,
- **STM32(NUCLEO-F103RB)** 가 모터/서보를 저수준 제어하고
- **MQTT** 로 GUI와 상태/명령/메타데이터를 주고받으며
- **QT GUI** 가 맵/카메라/로그/게이지를 실시간 시각화하는 구조이다.

### 전체 데이터 흐름
```
[ROS (SLAM/Planner 결과)]
│ (cmd_vel / map_state / pose 등)
▼
[Bridge Layer (MQTT + UART)]
│ │
│ UART Packet │ MQTT Topics
▼ ▼
[STM32 Firmware] [QT GUI]
(Motor/Servo) (Map/Camera/Logs/Gauge)
```


### 폴더별 역할 요약
- `Firmware/` : STM32 NUCLEO-F103RB(CubeIDE) 펌웨어. 모터/서보 제어 + UART 패킷 수신.
- `MQTT/` : 카메라/맵/모터 관련 브릿지(ROS 결과값 → MQTT/UART), HTTP 이미지 서빙 포함.
- `QT/` : MQTT 구독 기반 GUI. 맵 렌더링/카메라 표시/로그/게이지 통합.
- `ST Motor/` : 하드웨어 결선/핀맵/전원 구성 등 모터 시스템 물리 구성 문서화.
- `MQTT_Interface_Spec.md` : MQTT 토픽/JSON payload 명세(시스템 인터페이스 기준 문서).

> 각 폴더 내부에 상세 문서(md)가 별도로 존재하므로, 여기서는 “전체 구조” 관점으로만 요약한다.

---

## 2. 내가 기여한 핵심 파트

### 1) Firmware (STM32 / CubeIDE)
- DC Motor(속도/방향) + Servo(조향) 저수준 제어 구현
- UART 패킷 기반으로 명령을 안정적으로 수신/처리
- enable / emergency stop 등 제어 플래그를 포함한 안전한 동작 흐름 구성

### 2) MQTT 통신 및 Bridge 구성
- GUI ↔ 시스템 사이 통신을 MQTT 기반으로 설계/구현
- 카메라/맵 데이터는 **이미지 자체를 MQTT에 싣지 않고**, HTTP 서빙 + MQTT 메타(Url 등) 방식으로 경량화
- ROS에서 나온 결과값을 받아서
  - **STM32로 UART 패킷 전송**
  - 동시에 **GUI로 MQTT publish**
  형태로 동기화된 제어/모니터링 파이프라인 구축

### 3) QT GUI 개발
- MQTT 구독 기반의 통합 대시보드 구현
  - Map Viewer(맵 + shot/pose 오버레이)
  - Camera Viewer(HTTP 이미지 로드)
  - Logs(상태/이벤트 기록)
  - Gauge(속도 지표 시각화)
- UI 구성/표시 로직과 통신 로직을 분리하고, 실시간 갱신을 고려한 구조로 구현

---

## 3. Gaussian Splatting까지의 확장

본 프로젝트는 2D 지도/주행 파이프라인에 더하여
RC카가 촬영한 이미지와 위치 정보를 활용해 **Gaussian Splatting 기반 3D 맵**까지 연결하였다.

### 확장 흐름(개요)
```
[RC Car 이동 + Camera Shot + Pose 기록]
▼
[데이터셋 축적/전송]
▼
[Gaussian Splatting 3D 재구성]
▼
[3D 맵 결과 활용(시각화/후처리/확장)]
```

> 3D 재구성 파이프라인 및 데이터셋 구성/전송 방식은 관련 폴더/문서(md)에 상세히 정리되어 있다.
