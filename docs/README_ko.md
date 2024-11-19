# dynamixel-RDK

Dynamixel Sync Read & Write를 위한 ROS Development Kit.

<div align="center">

[English](../README.md) | [한국어](README_ko.md)
  
[![Build and Test - Humble](https://github.com/mjlee111/dynamixel-RDK/actions/workflows/humble.yml/badge.svg?branch=master&event=push)](https://github.com/mjlee111/dynamixel-RDK/actions/workflows/humble.yml)

</div>

## 요약
이 레포지토리는 [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)를 기반으로 한 Dynamixel Sync Read & Write를 위한 ROS2 패키지를 포함합니다.

## 기능
- Dynamixel의 **Sync Read** & **Sync Write** 지원
- Dynamixel의 **Bulk Read** & **Bulk Write** 지원
- 라이프사이클 노드 시스템 지원
- 파라미터를 위한 YAML 파일 지원
- 상태 및 제어 메시지 지원 - [Dynamixel RDK Messages](../dynamixel_rdk_msgs/README_ko.md)
- 간단한 GUI Dynamixel Controller - [Dynamixel RDK Manager](https://github.com/mjlee111/dynamixel_rdk_manager)

## 필수 요소
해당 패키지를 사용하기 위해서 다음 요소가 설치되어 있어야 합니다.

| Component | Version/Distribution | Notes |
|-----------|----------------------|-------|
| ROS2 |  Humble 이상 | 권장 ROS2 버전 |
| Dynamixel SDK | [github](https://github.com/ROBOTIS-GIT/DynamixelSDK) | Dynamixel 사용을 위한 Dynamixel SDK |

## 개발 환경

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill |

## 설치 방법
1. **Dynamixel SDK 설치** <br>
    [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) 참고

2. **레포지토리 clone**
    ```bash
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/mjlee111/dynamixel-RDK.git
    ```

3. **워크스페이스 빌드**
    ```bash
    $ cd ~/ros2_ws
    $ colcon build
    ```

4. **워크스페이스 소스 추가**
    ```bash
    $ source ~/ros2_ws/install/setup.bash
    ```

## 노드 실행 및 라이프사이클 노드 설정

1. **노드 실행** <br>
    `dynamixel_rdk_node`를 실행하기 위해서는 [dynamixel_rdk.launch.py](dynamixel_rdk_ros/launch/dynamixel_rdk.launch.py)를 사용합니다. 파라미터를 YAML 파일 또는 직접 명령어를 통해 로드할 수 있습니다.

    **예시 명령어:**
    ```bash
    $ ros2 launch dynamixel_rdk_ros dynamixel_rdk.launch.py
    ```
<br>

2. **라이프사이클 노드 설정** <br>
    라이프사이클 노드로 설정하기 위해서는 `ros2 lifecycle set` 명령어를 사용합니다.

    **노드 설정:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node configure
    ```
    노드가 설정되면 YAML 파일 또는 명령어를 통해 파라미터를 로드하고 Dynamixel 장치를 초기화합니다.

    **노드 활성화:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node activate
    ```
    노드가 활성화되면 Dynamixel의 토크를 **True**로 설정하고 Dynamixel 장치의 상태를 발행합니다.

    **노드 비활성화:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node deactivate
    ```
    노드가 비활성화되면 Dynamixel의 토크를 **False**로 설정합니다.

    **노드 종료:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node shutdown
    ```
    노드가 종료되면 시리얼 포트를 닫고 Dynamixel 장치를 해제합니다.

## 파라미터

노드는 YAML 파일 또는 명령어를 통해 설정할 수 있는 다양한 파라미터를 지원합니다. 파라미터는 다음과 같습니다.

### YAML 파일 파라미터
| 파라미터 이름             | 타입             | 기본값                  | 설명                                                                 |
|----------------------------|------------------|---------------------------------|-----------------------------------------------------------------------------|
| `device_port`              | `string`         | `/dev/ttyUSB0`                  | 장치가 연결된 시리얼 포트.                           |
| `baud_rate`                | `int`            | `1000000`                       | 시리얼 통신 속도.                                     |
| `dynamixels.ids`           | `array of int`   | `[1]`                     | 사용할 Dynamixel 모터 ID 목록.                                     |
| `dynamixels.types`         | `array of string`| `["MX"]`            | Dynamixel 모터 유형 목록 (예: "MX" for MX series).               |
| `dynamixels.max_position_limits` | `array of float` | `[3.14159]`  | 모터의 최대 위치 제한 목록.                  |
| `dynamixels.min_position_limits` | `array of float` | `[-3.14159]`| 모터의 최소 위치 제한 목록.                  |

### 예시 YAML 파일
세 개의 Dynamixel 모터를 위한 YAML 파일 - [dynamixel.yaml](dynamixel_rdk_ros/config/dynamixel.yaml)
```yaml
# ROS2 Parameters
/**:
    ros__parameters:
        device_port: "/dev/ttyUSB0"
        baud_rate: 1000000
        dynamixels:
            ids: [1, 2, 3]
            types: ["MX", "MX", "MX"]
            max_position_limits: [3.14159, 3.14159, 3.14159]
            min_position_limits: [-3.14159, -3.14159, -3.14159]
```

## 기여
모든 기여를 환영합니다! 버그 보고, 기능 제안 또는 풀 리퀘스트는 모두 피드백을 도와줍니다. 기여하고 싶다면 기여 가이드라인을 확인하거나 이슈를 제출하세요.

## 라이센스
이 프로젝트는 [Apache 2.0 License](LICENSE)로 라이센스됩니다. 라이센스 조건에 따라 자유롭게 사용하고 배포하세요.

## 연락처
질문이나 피드백이 있으면 언제든지 연락주세요! [menggu1234@naver.com][email]으로 연락 주세요.

[email]: mailto:menggu1234@naver.com
