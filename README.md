# UltrasonicRangingPlatform
samples three HC-SR04 ultrasonic sensors, one MPU-6050 IMU (± 2 g, ± 500 °/s) and drives two TowerPro SG90 micro-servos on a Raspberry Pi 4

exposes a single-client TCP API (ASCII commands + real-time telemetry) so any PC can (a) remotely configure a servo and (b) monitor distance, IMU data and servo angles in real time

boots entirely from a custom Buildroot image (< 40 MB ext4) with SSH, pigpio daemon, and our servo_daemon autostart script

demonstrates a 3-sample debounced motion rule: if the Z-axis acceleration is > +4 m/s² for three consecutive frames, the active servo steps clockwise; if < –4 m/s² it steps counter-clockwise; otherwise it holds position.
