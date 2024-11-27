# DYNAMIXEL

## Summary

* This is a folder to store the code to actuate Dynamixel.
* This folder includes [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) (submodule) and ROS 2 packages to control joint angles.

## Build and Usage

When using Dynamixel SDK for the first time, it is necessary to update the submodule.

```bash
cd ~/lbr_ws/src/LIMBERO
git submodule update --init
```

Next, build the package as follows:

```bash
cd ~/lbr_ws
colcon build
source install/setup.bash
```

## Trouble-Shooting

### Failed to enable torque

If you get the error message `[ERROR]: [Dynamixel] Failed to enable torque for Dynamixel ID ***`, you have to check PROTOCOL_VERSION or BAUDRATE is set properly.
The default BAUDRATE is 115200 but it may be wrong for LIMBERO.

If you cannot solve with above, try to update read/write permissions.

```bash
sudo chmod 666 /dev/ttyUSB0  # You may need to try "ttyACM0".
```

## Table of Dynamixel IDs

Labeled Number is defined for each Dynamixel in climbing robotics team, and it is different from assigned ID that is used for Dynamixel recognition.

| Limb | Which part? | Dynamixel model number | Labeled No.# | Assigned ID | Control mode |
|------|-------------|------------------------|---------------|-------------|-----|
| LF   | B2C         | XM540-W270-R           | #10           | 1           | Position |
| LF   | C2F         | XM540-W270-R           | #18           | 2           | Position |
| LF   | F2T         | XM430-W350-R           | #11           | 3           | Position |
| LF   | T2G         | XM430-W350-R           | #12           | 4           | Position |
| LF   | wristH      | 2XL430-W250-T          | #?            | 5           | Position |
| LF   | wristV      | 2XL430-W250-T          | #?            | 6           | Position |
| LF   | driving     | XM430-W350-R           | #?            | 7           | Velocity |
| LF   | Locking     | XL430-W250-T           | #?            | 8           | Velocity |
| LH   | B2C         | XM540-W270-R           | #24           | 9           | Position |
| LH   | C2F         | XM540-W270-R           | #25           | 10          | Position |
| LH   | F2T         | XM430-W350-R           | #28           | 11          | Position |
| LH   | T2G         | XM430-W350-R           | #27           | 12          | Position |
| LH   | wristH      | 2XL430-W250-T          | #?            | 13          | Position |
| LH   | wristV      | 2XL430-W250-T          | #?            | 14          | Position |
| LH   | driving     | XM430-W350-R           | #?            | 15          | Velocity |
| LH   | Locking     | XL430-W250-T           | #?            | 16          | Velocity |
| RH   | B2C         | XM540-W270-R           | #22           | 17          | Position |
| RH   | C2F         | XM540-W270-R           | #23           | 18          | Position |
| RH   | F2T         | XM430-W350-R           | #30           | 19          | Position |
| RH   | T2G         | XM430-W350-R           | #9            | 20          | Position |
| RH   | wristH      | 2XL430-W250-T          | #?            | 21          | Position |
| RH   | wristV      | 2XL430-W250-T          | #?            | 22          | Position |
| RH   | driving     | XM430-W350-R           | #?            | 23          | Velocity |
| RH   | Locking     | XL430-W250-T           | #?            | 24          | Velocity |
| RF   | B2C         | XM540-W270-R           | #20           | 25          | Position |
| RF   | C2F         | XM540-W270-R           | #21           | 26          | Position |
| RF   | F2T         | XM430-W350-R           | #26           | 27          | Position |
| RF   | T2G         | XM430-W350-R           | #29           | 28          | Position |
| RF   | wristH      | 2XL430-W250-T          | #?            | 29          | Position |
| RF   | wristV      | 2XL430-W250-T          | #?            | 30          | Position |
| RF   | driving     | XM430-W350-R           | #?            | 31          | Velocity |
| RF   | Locking     | XL430-W250-T           | #?            | 32          | Velocity |

## Author(s) and maintainer(s)

* Kazuki Takada (kazuki.takada.s2 at dc.tohoku.ac.jp)

## Reference

[1] [Dynamixel SDK - ROBOTIS e-Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
