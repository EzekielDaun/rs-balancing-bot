# rs-balancing-bot

![img](./example.gif)

A self balancing bot with STM32F103C8T6 and MPU6050, in Rust.

## Part List

- [Robot Kit](https://item.taobao.com/item.htm?spm=a1z10.1-c.w4004-10676576509.4.27b522d9dASwDW&id=45470143481)

- [Controller and Driving Circuit](https://item.taobao.com/item.htm?spm=a1z10.1-c.w4004-10676576509.28.27b522d9dASwDW&id=520789523491)

## Existing Problem

The robot takes the angle by reading the raw accelerometer in MPU6050 and comparing with the gravity. However when the robot is moving this result becomes unreliable and would make the PID tuning **EXTREMELY HARD**.

### Solutions

- The [dcmimu](https://crates.io/crates/dcmimu) crate applies an algorithm to calculate the position. However when it compiles, the binary is too large to be written into the flash.

- The Digital Motion Processor (DMP) integrated in the MPU6050 should be a better solution. It fuses the accelerometer and gyroscope data together to minimize the effects of errors inherent in each sensor. It is commonly used in many similar projects and is also chosen in the robot kit's example code. The provided Motion Driver includes the C usage. However, the existing Rust [crate](https://crates.io/crates/drogue-mpu-6050) doesn't works well with the STM32F1xx-HAL abstraction.

## Next Step

- [ ] Adapt DMP
- [ ] Split code into separate files
