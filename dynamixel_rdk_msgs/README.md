# Dynamixel RDK Messages
This package contains the ROS2 messages for the Dynamixel RDK. Messages are adapted from the [Dynamixel SDK Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/).

## Messages
### `DynamixelStatusMsgs`
Status message of the Dynamixel motor.

| Field | Type | Description | Unit |
|-------|------|-------------|------|
| header | std_msgs/Header | The header of the message. | - |
| id | uint8 | The ID of the Dynamixel motor. | - |
| torque_enabled | bool | Whether the torque is enabled. | - |
| error_status | uint8 | The error status of the Dynamixel motor. | - |
| present_position | float64 | The present position of the Dynamixel motor. | rad |
| present_velocity | float64 | The present velocity of the Dynamixel motor. | rad/s |
| present_acceleration | float64 | The present acceleration of the Dynamixel motor. | rad/s^2 |
| present_current | float64 | The present current of the Dynamixel motor. | mA |
| present_voltage | float64 | The present voltage of the Dynamixel motor. | V |
| present_temperature | float64 | The present temperature of the Dynamixel motor. | °C |

### `DynamixelMsgs`
Control message of the Dynamixel motor.

| Field | Type | Description | Unit |
|-------|------|-------------|------|
| header | std_msgs/Header | The header of the message. | - |
| goal_position | float64 | The goal position of the Dynamixel motor. | rad |
| profile_acceleration | float64 | The profile acceleration of the Dynamixel motor. | rad/s^2 |
| profile_velocity | float64 | The profile velocity of the Dynamixel motor. | rad/s |

### `DynamixelBulkReadMsgs`
Bulk read message of the Dynamixel motor.

| Field | Type | Description |
|-------|------|-------------|
| status_msgs | DynamixelStatusMsgs[] | The status messages of the Dynamixel motors. |

### `DynamixelBulkWriteMsgs`
Bulk write message of the Dynamixel motor.

| Field | Type | Description |
|-------|------|-------------|
| control_msgs | DynamixelMsgs[] | The control messages of the Dynamixel motors. |

## Disclaimer
- This package is based on the [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK), and the messages are adapted from the SDK.
- The error status of the Dynamixel motor is based on the [Dynamixel Protocol 2.0 status packet](https://emanual.robotis.com/docs/en/dxl/protocol2/#status-packet).
- `DynamixelBulkWriteMsgs` and `DynamixelMsgs` does not contains **id**. You need to push `DynamixelMsgs` to the `DynamixelBulkWriteMsgs` sequentially.

