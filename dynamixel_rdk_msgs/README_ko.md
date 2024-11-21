# Dynamixel RDK Messages
이 패키지는 Dynamixel RDK를 위한 ROS2 메시지를 포함합니다. 메시지는 [Dynamixel SDK Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/)에서 영감을 받았습니다.

## 메시지
### `DynamixelStatusMsgs`
Dynamixel 모터의 상태 메시지.

| Field | Type | Description | Unit |
|-------|------|-------------|------|
| header | std_msgs/Header | 메시지의 헤더. | - |
| id | uint8 | Dynamixel 모터의 ID. | - |
| torque_enabled | bool | 토크가 활성화되었는지 여부. | - |
| error_status | uint8 | Dynamixel 모터의 오류 상태. | - |
| present_position | float64 | Dynamixel 모터의 현재 위치. | rad |
| present_velocity | float64 | Dynamixel 모터의 현재 속도. | rad/s |
| present_acceleration | float64 | Dynamixel 모터의 현재 가속도. |
| present_current | float64 | Dynamixel 모터의 현재 전류. | mA |
| present_voltage | float64 | Dynamixel 모터의 현재 전압. | V |
| present_temperature | float64 | Dynamixel 모터의 현재 온도. | °C |
| min_max_position | float64[2] | Dynamixel 모터의 최소 및 최대 위치. | rad |

### `DynamixelMsgs`
Dynamixel 모터의 제어 메시지.

| Field | Type | Description | Unit |
|-------|------|-------------|------|
| header | std_msgs/Header | 메시지의 헤더. | - |
| goal_position | float64 | Dynamixel 모터의 목표 위치. | rad |
| profile_acceleration | float64 | Dynamixel 모터의 프로파일 가속도. | rad/s^2 |
| profile_velocity | float64 | Dynamixel 모터의 프로파일 속도. | rad/s |

### `DynamixelBulkReadMsgs`
Dynamixel 모터의 벌크 읽기 메시지.

| Field | Type | Description |
|-------|------|-------------|
| status_msgs | DynamixelStatusMsgs[] | Dynamixel 모터의 상태 메시지. |

### `DynamixelControlMsgs`
Dynamixel 모터의 벌크 쓰기 메시지.

| Field | Type | Description |
|-------|------|-------------|
| control_msgs | DynamixelMsgs[] | Dynamixel 모터의 제어 메시지. |

## 비고
- 이 패키지는 [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)를 기반으로 하며, 메시지는 SDK에서 영감을 받았습니다.
- Dynamixel 모터의 오류 상태는 [Dynamixel Protocol 2.0 상태 패킷](https://emanual.robotis.com/docs/en/dxl/protocol2/#status-packet)에서 영감을 받았습니다.
- `DynamixelControlMsgs` 및 `DynamixelMsgs`는 **id**를 포함하지 않습니다. `DynamixelMsgs`를 순차적으로 `DynamixelControlMsgs`에 추가해야 합니다.

