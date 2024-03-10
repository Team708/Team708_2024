package frc.robot;

import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//Command Imports
import frc.robot.commands.drive.SetRumble;
import frc.robot.commands.drive.DriveToAmp;
import frc.robot.commands.drive.ResetDrive;
import frc.robot.commands.drive.ResetGyro;
import frc.robot.commands.drive.ToggleDriveSpeed;
import frc.robot.commands.groups.IntakeNote;
import frc.robot.commands.intake.IntakeAllOut;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.PivotArm.armToParkShotAngle;
import frc.robot.commands.PivotArm.armToTrapShotAngle;
import frc.robot.commands.PivotArm.armToAmpShotAngle;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.SetShooterSpeedAmp;
import frc.robot.commands.shooter.SetShooterSpeedSpeaker;
import frc.robot.commands.AllSystemsOff;
import frc.robot.commands.AllSystemsOn;
import frc.robot.commands.ShootAmpSequence;
import frc.robot.commands.ShootSpeakerPCG;
import frc.robot.commands.ShootTrapSequence;
import frc.robot.commands.Feeder.FeederForward;
import frc.robot.commands.Feeder.FeederOff;
import frc.robot.commands.Feeder.FeederReverse;
import frc.robot.commands.shooter.ShooterReverse;
//Subsysem Imports
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.ArmConstants;

public class OI {

  // Gamepads
  public final static XboxController driverController = new XboxController(ControllerConstants.kDriverControllerPort); // Driver
  public final static XboxController operatorController = new XboxController(ControllerConstants.kOperatorControllerPort); // Operator
  public final static XboxController climberController  = new XboxController(ControllerConstants.kClimberControllerPort); // Climber
  public final static XboxController adaptiveGamepad = new XboxController(ControllerConstants.kAdaptiveControllerPort); // Adaptive
  
  public final static XboxController testController = new XboxController(4); // Driver 

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

  public static double getOperatorLeftY() {
  	return deadBand(operatorController.getLeftY(), ControllerConstants.kOperatorDeadBandLeftY);
  }

  // public static double getOperatorRightY() {
  // 	return deadBand(operatorController.getRightY(), ControllerConstants.kOperatorDeadBandRightY);
  // }

  public static double getClimberLeftY() {
  	return deadBand(climberController.getLeftY(), ControllerConstants.kClimberDeadBandLeftY);

  }

  // public static double getClimberRightY() {
  // 	return deadBand(climberController.getRightY(), ControllerConstants.kClimberDeadBandRightY);
  // }

  public static void configureButtonBindings(Drivetrain m_drive, Intake m_intake, Feeder m_feeder, Shooter m_shooter, PivotArm m_PivotArm) {

    //DRIVER//
    new JoystickButton(driverController, Button.kA.value) //TODO Change these buttons, current commands only for testing
    		.onTrue(new IntakeNote(m_intake, m_feeder));

    // new JoystickButton(driverController, Button.kB.value); //TODO Change these buttons, current commands only for testing
    		
    new JoystickButton(driverController, Button.kX.value)
        .onTrue(new AllSystemsOff(m_intake, m_feeder, m_shooter)); //TODO Change these buttons, current commands only for testing

    new JoystickButton(driverController, Button.kY.value)
        .whileTrue(new IntakeAllOut(m_intake, m_feeder))
    		.onFalse(new IntakeOff(m_intake));

    new JoystickButton(driverController, Button.kBack.value) //Fix
        .onTrue(new ResetGyro(m_drive))
        .onFalse(new SetRumble());

    new JoystickButton(driverController, Button.kLeftBumper.value)
        .onTrue(new ShootSpeakerPCG(m_drive, m_intake, m_feeder, m_PivotArm, m_shooter));
    
    new JoystickButton(driverController, Button.kRightBumper.value)
        .onTrue(new DriveToAmp(m_drive));
    
    new JoystickButton(driverController, Button.kLeftStick.value)
        .onTrue(new ToggleDriveSpeed(m_drive));
        
    //OPERATOR//
  
    new JoystickButton(operatorController, Button.kB.value)
        .onTrue(new FeederForward(m_feeder))
        .onFalse(new FeederOff(m_feeder));

    new JoystickButton(operatorController, Button.kY.value)
        .onTrue(new armToAmpShotAngle(m_PivotArm));
    
    new JoystickButton(operatorController, Button.kA.value)
        .onTrue(new armToParkShotAngle(m_PivotArm, ArmConstants.kParkAngle));

    new JoystickButton(operatorController, Button.kX.value)
    		.onTrue(new FeederReverse(m_feeder))
        .onTrue(new ShooterReverse(m_shooter))
        .onFalse(new FeederOff(m_feeder))
        .onFalse(new ShooterOff(m_shooter));

    new JoystickButton(operatorController, Button.kLeftBumper.value)     
      .onTrue(new SetShooterSpeedSpeaker(m_shooter))
      .onFalse(new ShooterOff(m_shooter));

    new JoystickButton(operatorController, Button.kRightBumper.value)
    		.onTrue(new ShootAmpSequence(m_drive, m_feeder, m_shooter, m_PivotArm));
    
    new JoystickButton(operatorController, Button.kRightStick.value)
    		.onTrue(new SetShooterSpeedAmp(m_shooter))
        .onFalse(new ShooterOff(m_shooter));

    new JoystickButton(operatorController, Button.kLeftStick.value)
        .onTrue(new armToParkShotAngle(m_PivotArm, ArmConstants.kDownAngle));

    new POVButton(operatorController, 0).onTrue(new armToParkShotAngle(m_PivotArm, 100.0));
    new POVButton(operatorController, 90).onTrue(new armToParkShotAngle(m_PivotArm, 45.0));
    new POVButton(operatorController, 180).onTrue(new armToParkShotAngle(m_PivotArm, 2.0));
    new POVButton(operatorController, 270).onTrue(new armToParkShotAngle(m_PivotArm, 68.0));

    
    //Adaptive Buttons
    new JoystickButton(adaptiveGamepad, Button.kA.value)
        .onTrue(new AllSystemsOn(m_intake, m_feeder, m_shooter));
    new JoystickButton(adaptiveGamepad, Button.kB.value)
        .onTrue(new AllSystemsOff(m_intake, m_feeder, m_shooter));

    //testing button
    // new JoystickButton(operatorController, Button.kRightStick.value)
    // 		.onTrue(new /*Command*/);
    
    //Climber//

    new JoystickButton(climberController, Button.kY.value)
    	.onTrue(new ShootTrapSequence(m_drive, m_feeder, m_shooter, m_PivotArm));

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

    new JoystickButton(testController, Button.kA.value) //TODO Change these buttons, current commands only for testing
    		.onTrue(new IntakeNote(m_intake, m_feeder));

    // new JoystickButton(driverController, Button.kB.value); //TODO Change these buttons, current commands only for testing
    		
    new JoystickButton(testController, Button.kX.value)
        .onTrue(new AllSystemsOff(m_intake, m_feeder, m_shooter));
  }
}