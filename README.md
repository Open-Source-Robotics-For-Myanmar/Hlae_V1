# Hlae_V1

An open source robot development.

## Hardware

The firmware targets an ESP32-WROOM DevKit V1 and two motor channels using the DRV8833-style IN1/IN2 PWM control used by the test sketch. Pin connections:

| Signal | Motor 1 / left | Motor 2 / right |
| --- | ---: | ---: |
| Encoder A | GPIO 18 | GPIO 25 |
| Encoder B | GPIO 19 | GPIO 26 |
| Driver IN1 | GPIO 32 | GPIO 27 |
| Driver IN2 | GPIO 33 | GPIO 14 |
| Driver STBY / nSLEEP | GPIO 4 | GPIO 13 |

Connect ESP32 ground and driver grounds together. Power motors from a suitable motor supply, not the ESP32 3.3 V pin. Confirm the exact driver breakout pin names before connecting it. DRV8833 ICs do not have a pin named STBY; many breakouts expose a sleep/enable pin, and some boards tie it active. Adapt the enable pins in the sketches to match the actual board. Do not connect GPIO 4 and GPIO 13 together unless the driver board wiring calls for it.

## Motor direction test

The earlier direction and encoder-count test is in `../Test_code/test_motors.ino`. Open it in Arduino IDE, select **ESP32 Dev Module** and the correct serial port, then upload. Open Serial Monitor at **115200 baud**.

The sketch runs Motor 1 forward and backward, then Motor 2 forward and backward. Each run lasts 2 seconds at PWM value 150 (about 59% of 255), prints the live count about every 100 ms, and prints the final count. It repeats the sequence after a 3-second pause. The count resets at the start of each direction run. With the encoder A/B wiring and ISR logic as shown, forward should increase the count and backward should decrease it. If a motor's signs are reversed, swap that motor's encoder A and B wires (or invert its count update).

## Closed-loop micro-ROS ROS 2 driver over Wi-Fi

`micro_ros_motor_driver/micro_ros_motor_driver.ino` implements differential drive with encoder feedback and per-wheel PID speed control. Robot settings are in `micro_ros_motor_driver/robot_config.h`; ordinary setup and tuning should not require editing the `.ino` file. It connects to a micro-ROS Agent over Wi-Fi UDP.

| ROS interface | Type | Meaning |
| --- | --- | --- |
| Subscribe `/cmd_vel` | `geometry_msgs/msg/Twist` | Uses `linear.x` and `angular.z` as m/s and rad/s targets |
| Publish `/odom` | `nav_msgs/msg/Odometry` | Integrated planar pose and measured linear/angular speed, `odom` → `base_link` frame IDs |

This provides standard ROS 2 topics directly, but the encoder odometry is open-loop with respect to robot pose: it does not fuse an IMU, correct wheel slip, or broadcast a TF transform. For Nav2, add a state estimation/TF component (commonly `robot_localization`) and configure its frames and odometry input. The PID coefficients and geometry defaults are examples, not calibrated values.

### Step-by-step setup

1. **Install ROS 2** on the computer that will run the Agent and your ROS nodes. Source the ROS 2 setup file in each terminal. Use a micro-ROS Agent image matching that ROS 2 distribution. The Docker example uses `kilted`; substitute the tag for your distribution.
2. **Install Arduino IDE and ESP32 board support.** Add the Espressif `esp32` package, select **ESP32 Dev Module**, and choose the USB port for your ESP32-WROOM DevKit V1.
3. **Install micro-ROS Arduino support.** Download a `micro_ros_arduino` ZIP release compatible with your ROS 2 distribution and install it with **Sketch → Include Library → Add .ZIP Library**. Follow the upstream release's supported ESP32 Arduino core versions; the current upstream project lists ESP32 Dev Module support. If using an incompatible library/core combination, compile or runtime errors can result.
4. **Set all board/network/robot values in `robot_config.h`:**
   - Replace `WIFI_SSID` and `WIFI_PASSWORD` with your Wi-Fi credentials.
   - Set `AGENT_IP_0` … `AGENT_IP_3` to the ROS computer's reachable LAN IPv4 address. The ESP32 and computer must be on a network that permits UDP traffic to `AGENT_PORT` (default 8888). Avoid guest Wi-Fi/client isolation.
   - Confirm motor driver pins/enable wiring and left/right motor assignments.
   - Measure wheel diameter, wheel-to-wheel track width, and encoder counts per wheel revolution. `ENCODER_COUNTS_PER_REV` must match the code's A-channel CHANGE counting convention (two A edges per A cycle); gear ratio and encoder location matter.
   - Correct `LEFT/RIGHT_MOTOR_SIGN` and `LEFT/RIGHT_ENCODER_SIGN` so a positive forward command makes both wheels roll forward and both signed encoder counts increase.
   - Tune the left/right `PID_KP`, `PID_KI`, and `PID_KD` constants. Start with low gains and a low `MAX_WHEEL_SPEED_MPS`; `PWM_MIN_MOVING` may compensate for measured motor deadband.
5. **Wire the motors and encoders** using the Hardware table. Join ESP32 and driver grounds. Use a separate suitable motor supply. Lift both wheels clear of the ground for first tests; keep an accessible hardware power switch.
6. **Open and upload** `micro_ros_motor_driver/micro_ros_motor_driver.ino`. The sketch reads `robot_config.h` in the same Arduino sketch folder. Do not keep Serial Monitor open while testing Wi-Fi; it is not the ROS transport.
7. **Start the UDP Agent** on the ROS computer. On Linux with Docker, after installing the Agent image, run:

   ```bash
   docker run -it --rm --net=host microros/micro-ros-agent:kilted udp4 --port 8888 -v6
   ```

   Ensure the host firewall allows inbound UDP port 8888. Use the same `AGENT_PORT` in the firmware. `--net=host` lets the Agent participate in the host ROS 2 DDS network on Linux.
8. **Verify topics** from another terminal with ROS 2 sourced:

   ```bash
   ros2 topic list
   ros2 topic echo /odom
   ros2 topic info /cmd_vel
   ```

   Wait for the ESP32 to join Wi-Fi and establish the Agent session. `/odom` should publish around 20 Hz.
9. **Test a small forward command with the wheels raised:**

   ```bash
   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.05}, angular: {z: 0.0}}"
   ```

   Check wheel direction and `/odom`. Stop the command publisher with Ctrl+C; the firmware should set target speed to zero after 500 ms. Then test a small reverse command and a slow in-place turn. Stop power if a wheel runs away or encoder feedback has the wrong sign. Only lower the robot after it holds speed smoothly and stops on command loss.

### PID and odometry calibration

- First set accurate `ENCODER_COUNTS_PER_REV`, `WHEEL_DIAMETER_M`, and `WHEEL_BASE_M`. Incorrect values directly distort speed feedback and odometry.
- Start with `KI=0` and `KD=0`. Increase `KP` until the wheel follows a low-speed target without sustained oscillation; then add a small `KI` to reduce steady speed error. Leave `KD=0` unless encoder noise and sample rate justify it. Tune each wheel separately.
- Verify speed using measured encoder ticks over a fixed interval and compare `/odom` distance with a measured floor distance. Tune `PWM_MIN_MOVING` only for genuine motor deadband.
- The firmware stops outputs when Wi-Fi is disconnected and times out stale `/cmd_vel` after 500 ms. Wi-Fi reconnection/session recovery may require an ESP32 reset depending on library/network behavior. Use a physical emergency stop; software stop is not a safety-rated function.

### References

- [micro_ros_arduino official repository](https://github.com/micro-ROS/micro_ros_arduino): supported boards, Wi-Fi transport examples, and release compatibility.
- [micro-ROS setup and Agent sources](https://github.com/micro-ROS/micro_ros_setup).
