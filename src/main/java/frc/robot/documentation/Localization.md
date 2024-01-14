<!-- Markdown language reference: https://www.markdownguide.org/basic-syntax/ -->
[Back To README](../../../../../../README.md)

# Localization
Localization is the ability of a robot to determine its position and orientation within its environment. This is a critical aspect of robotic systems, as it enables the robot to navigate and perform tasks effectively.

### Odometry:
This method uses sensors to measure the movement of the robot's wheels. By keeping track of the distance and direction traveled, the robot can estimate its position. However, odometry is prone to cumulative errors over time, and it may not provide accurate localization over long distances or fast speeds.

### Inertial Navigation:
Inertial measurement units (IMUs) are used to measure acceleration and angular velocity, allowing the robot to estimate its position and orientation. Inertial navigation is most acurate over short periods of time and is often combined with other localization methods to improve accuracy.

### Landmark-based Localization:
Beacons or markers, such as AprilTags, are placed in the environment around the robot, and the robot uses sensors, such as cameras, to detect and triangulate its position relative to these markers. They provide a fixed and acurate reference point for calculating position but detection is slower than using inerital meaurment or odometry and can be hard to see if the markers can be blocked by obsticals or the robot is moving fast.

### Sensor Fusion:
This method combines different sensors and algorithms to improve localization accuracy. Sensor fusion involves integrating data from multiple sources, such as odometry, IMUs, cameras, and LIDAR, to obtain a more reliable and accurate estimate of the robot's pose. On the robot, the pose estimation method takes the robot's last position (pose) stored and estimates the current state based on the change in odometry and IMU measurments since that last position. The value stored until the method is called again a short time later. The robot pose is updated as soon as a new pose can be calcuated though triangulation between the camera and the AprilTags.

## Vision

- [Limelight 3](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary)
    - [AprilTags](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags)
    - [Limelight Map Builder Tool](https://tools.limelightvision.io/map-builder)

## Pose Estimation

- [Pose Estimation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html)

## Driving While Shooting

1. Use pose estimation to get robot position relative to field
2. Allow driver to translate robot but give control to targeting software for controlling rotation for aiming
3. Driver allows the robot to shoot when ready using a controller button. Robot use position on the field to determine whether the amp or the speaker is closer and locks onto the respective target.
    - This allows for using the same button press for aligning to both targets.
    - Provide haptic feedback for when the targeting is enabled.
4. Robot calculates game peice trajectory and shoots when the shot in line with with target.
    - targeting class accesses shooter subsystem

