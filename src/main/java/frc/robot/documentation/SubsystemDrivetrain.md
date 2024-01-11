<!-- Markdown language reference: https://www.markdownguide.org/basic-syntax/ -->
[Back To README](../../../../../../README.md)

# Drivetrain Subsystem

The drivetrain subsystem is the high level controller for the swerve based drivetrain. It calculates the states of each swerve module and passes those values into the swerve module class. Each swerve module class object converts the states into a wheel direction and speed for the motor controllers of its respective module.

Swerve drive is a type of holonomic drivetrain used in robotics. Unlike traditional drivetrains that rely on differential steering or tank drive, swerve drive provides exceptional maneuverability and agility. Here's an explanation of the basic principles of a swerve drive:

### Independent Wheel Movement:

In a swerve drive system, each wheel has its own dedicated motor and can be independently controlled in terms of both speed and direction. This is a key feature that distinguishes swerve drive from other drivetrain configurations.

### Modules:

Swerve drive systems typically consist of modules, each comprising a motor, a wheel, and a swiveling mechanism. The swiveling mechanism allows the wheel to rotate or "swerve" independently of the robot's main frame.

### Holonomic Motion:

Holonomic motion refers to a robot's ability to move in any direction instantaneously. Since each wheel can be independently controlled in both speed and direction, a swerve drive-equipped robot can move forward, backward, sideways, and rotate on the spot with great precision. This precise maneuverability is essential in robotics competitions where robots need to navigate obstacles, pick up game pieces, and interact with other robots.

### Control System:

Swerve drive systems require sophisticated control algorithms to manage the individual wheel movements effectively. These algorithms involve calculations to determine the necessary speed and direction for each wheel to achieve the desired overall motion of the robot and prevent unnessesary scrubbing of the wheels that leads to lower performane and high energy consumption.

### Encoders and Sensors:

Encoders and sensors are often used to provide feedback on the position and speed of each wheel. This information is crucial for the control system to maintain accurate control and ensure that the robot moves as intended.

## Hardware Specifications

**Motors:** REV Robotics [NEO Brushless Motor](https://www.revrobotics.com/rev-21-1650/)

**Motor Controllers:** REV Robotics [CAN Spark Max](https://www.revrobotics.com/rev-11-2158/)

**Turning Encoder:** CTRE [Mag Encoder](https://store.ctr-electronics.com/srx-mag-encoder/)

## Mechanical Specifications

**Swerve Modules:** Swerve Drive Specialties [MK4i L2](https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777303153)

- Gear Ratio, Drive: (425/53):1
  - First Stage: 14:50
  - Second Stage: 27:17
  - Third Stage: 15:45
