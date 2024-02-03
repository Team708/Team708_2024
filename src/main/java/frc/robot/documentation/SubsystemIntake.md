<!-- Markdown language reference: https://www.markdownguide.org/basic-syntax/ -->
[Back To README](../../../../../../README.md)

# Intake Subsystem

The intake picks up game pieces off the ground with a series of rollers from under the bumper. Four independantly controllable roller group assemblies allow intaking on all sides of the robot while also providing the ability to vector the game pieces in and out of different sides of the robot for more complex autos. The ground and direction of roller rotation are used to direct pieces to appropriate locations.

Once intook, the game pieces, called notes, are processed by the feeder subsystem.


### Individual Intake Assemblies

The side and back intakes are virtually identical; they are rotated 90° with a NEO Brushless Motor located on the left side. The front intake integrates into a conveyor system and is powered by a single motor that is located in the back left of the intake subsystem. These systems converge to allow for intake from all sides and expulsion from the front and back intakes. 

### Sensors:

Diffuse sensors are a type of optical sensor that are immune to ambient light. A total of five sensors are present. Each roller group assembly has a sensor located in the right corner near the swerve module when an observer is oriented to look into the robot. Each of these sensors is oriented to the next counterclock/clockwise??? swerve module. When an intook note is detected by the sensor, the note is indexed and the is subsequently used for data logging and note location tracking. The l/r/f/b??? intake assembly has an additional sensor that is pointed towards the center of the robot and indexes notes sitting in the center of the overall intake assembly. 

**Diffuse Sensor:** [Allen-Bradley 42EF-D2MPAK-F4](https://www.rockwellautomation.com/en-us/products/details.42EF-D2MPAK-F4.html)



## Hardware Specifications

**Motors:** REV Robotics [NEO Brushless Motor](https://www.revrobotics.com/rev-21-1650/)

**Motor Controllers:** REV Robotics [CAN Spark Max](https://www.revrobotics.com/rev-11-2158/)

## Mechanical Specifications
- 4 independent REV Neo motors, one per side
- Gear Ratio: (6/11):1
  - First Stage: 12:22
- Roller Diameter: 1.875"
