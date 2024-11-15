# dynamixel-RDK

Dynamixel ROS Development Kit for Sync Read and Write. 

<div align="center">
  
[![Build and Test - Humble](https://github.com/mjlee111/dynamixel-RDK/actions/workflows/humble.yml/badge.svg?branch=master&event=push)](https://github.com/mjlee111/dynamixel-RDK/actions/workflows/humble.yml)

</div>

## Overview
This repository contains a ROS2 package for Dynamixel Sync Read and Write. Supports **Lifecycle Node** system.

## Requirements
To use the packages in this repository, make sure you have the following installed:

| Component | Version/Distribution | Notes |
|-----------|----------------------|-------|
| ROS2 |  Humble or higher | Recommended ROS2 distributions |
| Dynamixel SDK | [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) | Dynamixel SDK for controlling Dynamixel  |

## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill |

## Installation
1. **Install Dynamixel SDK** <br>
    Please refer to the [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) for installation.

2. **Clone this repository**
    ```bash
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/mjlee111/dynamixel-RDK.git
    ```

3. **Build the workspace**
    ```bash
    $ cd ~/ros2_ws
    $ colcon build
    ```

4. **Source the workspace**
    ```bash
    $ source ~/ros2_ws/install/setup.bash
    ```

## Launching the node & Set Lifecycle Node

1. **Launch the node** <br>
    To run the `dynamixel_rdk_node`, use the provided [launch file](dynamixel_rdk_ros/launch/dynamixel_rdk.launch.py). It supports loading parameters from a YAML file or directly through launch arguments.

    **Example launch command:**
    ```bash
    $ ros2 launch dynamixel_rdk_ros dynamixel_rdk.launch.py
    ```

2. **Set Lifecycle Node** <br>
    To set the node as a lifecycle node, you can use the `ros2 lifecycle set` command.

    **Configure the node:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node configure
    ```
    When the node is configured, it will load the parameters from the YAML file or launch arguments. Also, it will initialize the Dynamixel devices(Set ID, Baud Rate, etc.).

    **Activate the node:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node activate
    ```
    When the node is activated, it will set dynamixel's torque to **True** and start to publish the status of the Dynamixel devices. 

    **Deactivate the node:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node deactivate
    ```
    When the node is deactivated, it will set dynamixel's torque to **False**.

    **Shutdown the node:**
    ```bash
    $ ros2 lifecycle set dynamixel_rdk_node shutdown
    ```
    When the node is shutdown, it will close the serial port and release the Dynamixel devices. 

## Parameters

The node supports various parameters that can be configured via a YAML file or command line arguments. Here's a table of parameters:

| Parameter Name             | Type             | Default Value                  | Description                                                                 |
|----------------------------|------------------|---------------------------------|-----------------------------------------------------------------------------|
| `device_port`              | `string`         | `/dev/ttyUSB0`                  | The serial port to which the device is connected.                           |
| `baud_rate`                | `int`            | `1000000`                       | The baud rate for serial communication.                                     |
| `dynamixels.ids`           | `array of int`   | `[1]`                     | List of Dynamixel motor IDs to be used.                                     |
| `dynamixels.types`         | `array of string`| `["MX"]`            | List of types of Dynamixel motors (e.g., "MX" for MX series).               |
| `dynamixels.max_position_limits` | `array of float` | `[3.14159]`  | List of maximum position limits for the motors in radians.                  |
| `dynamixels.min_position_limits` | `array of float` | `[-3.14159]`| List of minimum position limits for the motors in radians.                  |


## Contributing
I welcome all contributions! Whether it's bug reports, feature suggestions, or pull requests, your input helps me to improve. If you're interested in contributing, please check out my contributing guidelines or submit an issue.

## License
This project is licensed under the [Apache 2.0 License](LICENSE). Feel free to use and distribute it according to the terms of the license.

## Contact
If you have any questions or feedback, don't hesitate to reach out! You can contact me at [menggu1234@naver.com][email].

[email]: mailto:menggu1234@naver.com
