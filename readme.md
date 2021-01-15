#rs974 4B25 project: quadcopter flight control 

This program runs on the iFlight SucceX micro F4 flight controller (uses an STM32F411CE microcontroller and MPU6000 IMU).

## Installation

Once four motors and an SBUS receiver are connected as in the wiring diagram in https://shop.iflight-rc.com/image/catalog/download/SucceX/SucceX%20micro%20F4%20V2.1%20wiring%20diagram-200604.png, place the MCU in DFU mode and upload the code using a tool such as dfu-util.

## Usage

The PID gains for each axis can be tuned in the declaration of private variables on top of main.c. The PID looptime can be adjusted by adjusting the delay in the main function. Note that the angular velocity reference for all axes is hard-set to 0 in this basic implementation, and that the pitch axis stick has to be held up to keep the quadcopter armed, for safety. 

Further developments include fine-tuning of the PID gains to ensure stability, and measurement of the pitch/roll/yaw sticks to set the reference angular velocities. Furthermore, a timer can be used to call the PID loop at set intervals rather than using delays in the main loop.

Attitude-based control can also be implemented by creating 3 additional PID control loops and taking accelerometer readings from the MPU6000. This however requires conversion from the body reference frame to a fixed reference frame using either Euler angles or quaternions.

