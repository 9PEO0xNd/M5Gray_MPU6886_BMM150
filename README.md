# M5Gray_MPU6886_BMM150 for arduino IDE

This is M5Stack Gray MPU6886+BMM150 examples for arduino IDE.

`BMM150class.cpp` and `BMM150class.h` were forked from omegatao's repository. Original link (platform IO based)
https://github.com/omegatao/M5StackGrey_MPU6886_BMM150_AHRS_sample

## What's New ?
- Added soft iron distortion correction.
>[Simple and Effective Magnetometer Calibration](https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration)
>Kris Winer edited this page on 15 Aug 2017 · 17 revisions

- Added gyro offset calibration. However, built in `M5.IMU.setGyroOffset(Ωx0,Ωy0,Ωz0)` is not used.
  It starts automatically. (still has gyro drift. but reduced a little)
- yaw tweaked 90 degrees (button A,B,C are front side)

## Usages
- Copy BMM150 folder into your arduino libraries folder.
- Some .ino files are required lovyanGFX and lovyanlancher.
- M5Stack-SD-Updater ~~version must be 0.5.2 (1.x or higher will not work in my code. I don't know why) 1.0.x is OK now.~~ Verified arduino 1.8.15
- tilt_compass.ino is traditional tilt-compensated compass. Not using 9-AXIS quaternion.
- Do not move or tilt your M5Stack during the gyro offset calibration.
- Do not use this unit in situations involving safety to life.
- ~~- Not supported M5Unified library. (Consider M5Stack.h library depandency such as I2C, etc.)~~
- Added example code using M5Unified library.(2023年の恵方を入れました)

## References
- https://github.com/BoschSensortec/BMM150-Sensor-API
- https://github.com/m5stack/M5_BMM150
- https://github.com/m5stack/M5Unified
- https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

## Revision history
- 2021-Jan Initial release.
- 2021-Feb updated Readme.md, added built in sensor's orientation images.
- 2023-Jan Modified I2C part and added example code using M5Unified library.

Have fun !
