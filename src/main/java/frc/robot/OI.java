package frc.robot;

import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Command Imports
import frc.robot.commands.drive.DriveToAmp;
import frc.robot.commands.groups.IntakeNote;
import frc.robot.commands.intake.IntakeAllIn;
import frc.robot.commands.intake.IntakeAllOut;
import frc.robot.commands.intake.IntakeEjectBack;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.PivotArm.EnableArmAutoAim;
import frc.robot.commands.PivotArm.armToFartherShotAngle;
import frc.robot.commands.PivotArm.armToParkShotAngle;
import frc.robot.commands.PivotArm.armToAmpShotAngle;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.SetShooterSpeedAmp;
import frc.robot.commands.shooter.SetShooterSpeedBumperShot;
import frc.robot.commands.ShootAmpSequence;
import frc.robot.commands.ShootSpeakerBumperShotSCG;
import frc.robot.commands.Feeder.FeedNoteForStorage;
import frc.robot.commands.Feeder.FeedNoteToShoot;
//Subsysem Imports
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Shooter;

public class OI {

  // Gamepads
  public final static XboxController driverController = new XboxController(ControllerConstants.kDriverControllerPort); // Driver
  public final static XboxController operatorController = new XboxController(ControllerConstants.kOperatorControllerPort); // Operator
  // public final static XboxController climberController  = new XboxController(ControllerConstants.kClimberControllerPort); // Climber
  // public final static XboxController adaptiveGamepad = new XboxController(ControllerConstants.kAdaptiveControllerPort); // Adaptive

  /*
   * Driver JoystickButton
   */

  public OI() {

  }

  private static double deadBand(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static double getDriverLeftX() {
    return deadBand(driverController.getLeftX(), ControllerConstants.kDriverDeadBandLeftX);
  }

  public static double getDriverRightX() {
    return deadBand(driverController.getRightX(), ControllerConstants.kDriverDeadBandRightX);
  }

  public static double getDriverLeftY() {
    return deadBand(driverController.getLeftY(), ControllerConstants.kDriverDeadBandLeftY);
  }

  public static double getDriverRightY() {
    return deadBand(driverController.getRightY(), ControllerConstants.kDriverDeadBandRightY);
  }

  // public static double getOperatorLeftX() {
  // 	return deadBand(operatorController.getLeftX(), ControllerConstants.kOperatorDeadBandLeftX);
  // }

  // public static double getOperatorRightX() {
  // 	return deadBand(operatorController.getRightX(), ControllerConstants.kOperatorDeadBandRightX);
  // }

  // public static double getOperatorLeftY() {
  // 	return deadBand(operatorController.getLeftY(), ControllerConstants.kOperatorDeadBandLeftY);
  // }

  // public static double getOperatorRightY() {
  // 	return deadBand(operatorController.getRightY(), ControllerConstants.kOperatorDeadBandRightY);
  // }

  // public static double getClimberLeftY() {
  // 	return deadBand(climberController.getLeftY(), ControllerConstants.kClimberDeadBandLeftY);

  // }

  // public static double getClimberRightY() {
  // 	return deadBand(climberController.getRightY(), ControllerConstants.kClimberDeadBandRightY);
  // }

  public static void configureButtonBindings(Drivetrain m_drive, Intake m_intake, Feeder m_feeder, Shooter m_shooter, PivotArm m_PivotArm) {

    //DRIVER//
    new JoystickButton(driverController, Button.kA.value) //TODO Change these buttons, current commands only for testing
    		.onTrue(new IntakeNote(m_intake, m_feeder));

    new JoystickButton(driverController, Button.kB.value) //TODO Change these buttons, current commands only for testing
    		.onTrue(new IntakeAllOut(m_intake))
    		.onFalse(new IntakeOff(m_intake));

    new JoystickButton(driverController, Button.kX.value) //TODO Change these buttons, current commands only for testing
    		.whileTrue(new FeedNoteForStorage(m_feeder));

    new JoystickButton(driverController, Button.kY.value)
        .whileTrue(new FeedNoteToShoot(m_feeder, m_PivotArm));

    new JoystickButton(driverController, Button.kRightBumper.value)
     		.toggleOnTrue(new DriveToAmp(m_drive));
    
    // new JoystickButton(driverController, Button.kStart.value)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(driverController, Button.kBack.value)
    // 		.onTrue(new /*Command*/)
    // 		.onTrue(new /*Command*/);

    new JoystickButton(driverController, Button.kLeftBumper.value)
    		//.whileHeld((new AutoTargetSpeaker(m_drive)));
        .whileTrue(new ShootSpeakerBumperShotSCG(m_drive, m_feeder, m_shooter, m_PivotArm));
        // .whileFalse(new DisableAutoTargetSpeaker(m_drive));
    
    

    // new JoystickButton(driverController, Button.kRightStick.value)
    // 		.onTrue(new /*Command*/);
        
    // new JoystickButton(driverController, Button.kLeftStick.value)
    // 		.onTrue(new /*Command*/);
      
    // new DPadButton(driverController, DPadButton.Direction.UP)
    // 		.onTrue(new /*Command*/);
    
    // new DPadButton(driverController, DPadButton.Direction.DOWN)
    // 		.whileTrue(new /*Command*/);
    
    
    //OPERATOR//
    
    //testing button
    new JoystickButton(operatorController, Button.kA.value)
    .onTrue(new SetShooterSpeedBumperShot(m_shooter))
    .onFalse(new ShooterOff(m_shooter));

    // //testing button
    // new JoystickButton(operatorController, Button.kB.value)
    // .onTrue(new SetShooterSpeedAmp(m_shooter))
    // .onFalse(new ShooterOff(m_shooter));

    // //testing button
    // new JoystickButton(operatorController, Button.kA.value)
    //     .onTrue(new armToParkShotAngle(m_PivotArm));

    //testing button
    new JoystickButton(operatorController, Button.kB.value)
    .onTrue(new armToFartherShotAngle(m_PivotArm));
    
    // new JoystickButton(operatorController, Button.kX.value)
    // 		.onTrue(new /*Command*/)
    // 		.whenReleased(new /*Command*/);
    
    //testing button
    new JoystickButton(operatorController, Button.kY.value)
        .onTrue(new armToAmpShotAngle(m_PivotArm));

    // new JoystickButton(operatorController, Button.kStart.value)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(operatorController, Button.kBack.value)
    // 		.whileTrue(new /*Command*/);

    //testing button
    new JoystickButton(operatorController, Button.kLeftBumper.value)
        .whileTrue(new EnableArmAutoAim(m_PivotArm,m_drive));

    //testing button
    new JoystickButton(operatorController, Button.kRightBumper.value)
    		.whileTrue(new ShootAmpSequence(m_drive, m_feeder, m_shooter, m_PivotArm));
    
    new JoystickButton(operatorController, Button.kRightStick.value)
    		.onTrue(new SetShooterSpeedAmp(m_shooter));

    //testing button
    // new JoystickButton(operatorController, Button.kRightStick.value)
    // 		.onTrue(new /*Command*/);
    
    //Climber//

    // new JoystickButton(climberController, Button.kY.value)
    // 	.onTrue(new /*Command*/);

    // new JoystickButton(climberController, Button.kA.value)
    // 	.onTrue(new /*Command*/);
    
    // new JoystickButton(climberController, Button.kStart.value)
    // 	.onTrue(new /*Command*/);
    
    // new JoystickButton(climberController, Button.kBack.value)
    // 	.onTrue(new /*Command*/);
    
    // new JoystickButton(climberController, Button.kB.value)
    // 	.onTrue(new /*Command*/);	
    
    // new JoystickButton(climberController, Button.kX.value)
    // 	.onTrue(new /*Command*/);	
    
    // new JoystickButton(climberController, Button.kRightBumper.value)
    // 	.onTrue(new /*Command*/);
    
    //Adaptive Controller

    // new JoystickButton(adaptiveGamepad, Button.kStart.value)
    // 	.onTrue(new /*Command*/);
    
    // new JoystickButton(adaptiveGamepad, Button.kBack.value)
    // 	.onTrue(new /*Command*/);
      
    // new JoystickButton(adaptiveGamepad, Button.kB.value)
    // 	.onTrue(new /*Command*/);
    
    // new JoystickButton(adaptiveGamepad, Button.kA.value)
    // 	.onTrue(new /*Command*/);
  }
}