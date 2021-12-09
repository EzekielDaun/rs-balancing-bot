# rs-balancing-bot

![img](./example.gif)

A self balancing bot with STM32F103C8T6 and MPU6050, in Rust.

The project is intended to show the high expressiveness and library management of the Rust programming language, rather than focusing on the bot's performance.

## Part List

- [Robot Kit](https://item.taobao.com/item.htm?id=45470143481)

- [Controller and Driving Circuit](https://item.taobao.com/item.htm?&id=520789523491)

## Existing Problems

The robot takes the angle by reading the raw accelerometer in MPU6050 and comparing with the gravity. However when the robot is moving this result becomes unreliable and would make the PID tuning **EXTREMELY HARD**.

### Solutions

- A first-order complementarity filter like: angle = k*(angle_acc+gyro*dt)+(1-k)\*angle, could be easily implemented.

- The [dcmimu](https://crates.io/crates/dcmimu) crate applies an algorithm to calculate the orientation. However when it compiles, the binary is too large to be written into the flash.

- The [adskalman](https://crates.io/crates/adskalman) crate implements a Kalman filter, which can be used to calculate the orientation.

- The Digital Motion Processor (DMP) integrated in the MPU6050 should be a better solution. It fuses the accelerometer and gyroscope data together to minimize the effects of errors inherent in each sensor, without increasing the load on MCU. It is commonly used in many similar projects and is also chosen in the robot kit's example code. The provided Motion Driver includes the C usage. However, the existing Rust [crate](https://crates.io/crates/drogue-mpu-6050) doesn't works well with the STM32F1xx-HAL abstraction.
