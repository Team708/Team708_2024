<!-- Markdown language reference: https://www.markdownguide.org/basic-syntax/ -->
[Back To README](../../../../../../README.md)

# Robot Bring Up

### Drivetrain:
- [ ] 1. Verify SparkMax CAN IDs and settings
- [ ] 2. Verify CANCoder CAN IDs and settings
- [ ] 3. Verify swerve module drive and turn gear ratios
- [ ] 4. Set turn offsets 
- [ ] 5. while robot is off the ground, verify motor directions and wheels steer correctly
- [ ] 6. Verify drivebase drives on carpet without issue
- [ ] 7. Verify robot pose matches the physical robot
- [ ] 8. Verify the Pigeon is working correctly
- [ ] 9. Test basic autos work correctly (DogLeg and ReverseDogLeg)

### Intake:
- [ ] 1. Verify SparkMax CAN IDs and settings
- [ ] 2. Run the intake commands via the buttons and verify motors rotate in the correct direction.
- [ ] 3. Set appropriate roller speeds once the feeder and shooter are configured

### Feeder:
- [ ] 1. Verify SparkMax CAN IDs and settings
- [ ] 2. Verify motors run in the correct direction
- [ ] 3. Insert a note manually and verify feeder sensors work
- [ ] 4. Remove the note and verify indexing logix works as expected with the sensors and rollers
- [ ] 3. Set appropriate roller speeds once the shooter is configured

### Arm:

- [ ] 1. Verify SparkMax CAN IDs and settings
- [ ] 2. Set motor to coast
- [ ] 3. Display remote encoder, motor encoder, and arm angle values to the smartdashboard
- [ ] 4. Verify values are within limits and moving in correct directions
- [ ] 5. Set absolute encoder offset value based on arm being horizontal. Set value directly as seen (i.e. value reads -17, set offset as -17)
- [ ] 6. Set motor to brake
- [ ] 7. Verify motor direction is correct. SLOWWW!!! Current limit set to 5A
- [ ] 8. Verify the hard limit switch works
- [ ] 9. Set soft limits
- [ ] 10. Set arm position constants
- [ ] 11. Set PID vaules
- [ ] 12. Verify arm can start from starting config (bumper shot)

### Shooter:
- [ ] 1. Verify SparkMax CAN IDs and settings
- [ ] 2. Verify motors run in the correct direction
- [ ] 3. Run the shooter commands via the buttons and verify motors rotate in the correct direction.
- [ ] 4. tune the PID and verify the shooter gets up to the apropriate speeds and is stable

### Note Path Integration:
- [ ] 1. After the intake, feeder, shooter, and arm are functional, verify without a note that the intake and feeder turn off when the feeder sensor is flagged
- [ ] 2. While empty, Verify the shooter can spin up and request the feeder to run while the sensors are both active and not
- [ ] 3. Verify the intake collect a note from the floor and pass the it into the feeder. This should happen smoothly and the note should stop when it reaches the end of the feeder but before entering the shooter
- [ ] 4. Verify the shooter can shoot a note out the speaker side from the feeder
- [ ] 5. Verify the shooter can shoot a note out the Amp side from the feeder
- [ ] 6. Verify a second note can enter the system without jamming
- [ ] 7. Verify Arm can go to the correct position and the shooter can shoot the note for bumper the speaker bumper shot and the Amp shot

### SemiAutos:
- [ ] 1. Verify aming while driving works. The robot should turn to always face the target and the arm should adjust its angle based on its distance to the target
- [ ] 2. Verify the target changes when the alliance color changes
- [ ] 3. Verify drive to Amp works and shoot works

### Autos:
- [ ] 1. 



