<!-- Markdown language reference: https://www.markdownguide.org/basic-syntax/ -->
[Back To README](../../../../../../README.md)

# Commands

## Autos

- [DoNothing](../commands/auto/doNothingCommand.java): Does nothing
- FiveBall: Shoots five NOTES
- ShootOnePlusThreeClose: Shoots one preloaded, picks up and shoots the three close NOTES
- PukeFive: Shoot preloaded and move five far notes toward WING without notes crossing WING line
- DriveStraight: Drives Straight
- ShootPreloadDriveStraight: Shoots preloaded and drives straight

## Drivetrain
- EnableAutoTargetSpeaker
- DriveToPose

## Intake
- [IntakeAllIn](../commands/intake/IntakeAllIn.java)
- [IntakeAllOut](../commands/intake/IntakeAllOut.java)
- [IntakeEjectBack](../commands/intake/IntakeEjectBack.java)
- [IntakeEjectFront](../commands/intake/IntakeEjectFront.java)
- [IntakeOff](../commands/intake/IntakeOff.java)

## Feeder
- FeederFeedShooter
- FeederStagePieces
    - If empty, sowes 1 NOTE into 2nd stage
    - If 1 NOTE staged, stages 2nd NOTE into 1st stage
    - IF NOTES in both stages, Return Full
- FeederReverse

## Shooter
- ShooterShootSpeaker
    - Shooter.SetSpeedSpeaker(), Return OK to shoot
    - FeederFeedShooter
- ShooterShootAmp
    - ShooterArmToAmp
    - Shooter.SetSpeedAmp(), Return OK to shoot
    - FeederFeedShooter
- ShooterOff
    - Shooter.Off()
- ShooterClearJam
    - Shooter.Reverse()
    - Run for 500ms
    - Shooter.Off()
- ShooterArmToPosition
    - Shooter.SetrmAngel()
- ShooterArmToPark
    - Shooter.SetArmAngle(Park Angle)
- ShooterArmToSpeakerBumper
    - Shooter.SetArmAngle(Bumper Shot Angle)
- ShooterArmToAmp
    - Shooter.SetArmAngle(Amp Angel)
- ShooterArmToExtent
    - Shooter.SetArmAngle(Extended Angle)
- ShooterArmToClimb
    - Shooter.Climb()

## Command Groups
- AutoShootSpeaker
    - EnableAutoTargetSpeaker
    - ShooterShootSpeaker
- AutoShootAmp
    - DriveToPose(Pose)||ShooterArmToAmp
    - Shooter.SetSpeedAmp(), Return OK to shoot
    - FeederFeedShooter
- AutoDriveToFeeder
    - DriveToPose(Pose)