package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.shooter.ShootSpeakerBumperShot;
import frc.robot.commands.shooter.armToBumperShotAngle;
import frc.robot.commands.shooter.armToFartherShotAngle;
import frc.robot.commands.shooter.armToPodiumShotAngle;
import frc.robot.commands.shooter.ShooterOff;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.XboxController.Button;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Subsysem Imports
import frc.robot.subsystems.drive.Drivetrain;
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

  public static void configureButtonBindings(Drivetrain m_drive, Intake m_intake, Shooter m_shooter) {

    //DRIVER//

    // new JoystickButton(driverController, Button.kA.value)
    // 		.onTrue(new /*Command*/);	

    // new JoystickButton(driverController, Button.kB.value)
    // 		.whileTrue(new /*Command*/)
    // 		.whenReleased(new /*Command*/);

    // new JoystickButton(driverController, Button.kX.value)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(driverController, Button.kY.value)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(driverController, Button.kRightBumper.value)
    // 		.onTrue(() -> /*Command*/)
    // 		.whenReleased(() -> /*Command*/);
    
    // new JoystickButton(driverController, Button.kStart.value)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(driverController, Button.kBack.value)
    // 		.onTrue(new /*Command*/)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(driverController, Button.kLeftBumper.value)
    // 		.onTrue(() -> /*Command*/)
    // 		.whenReleased(() -> /*Command*/);

    // new JoystickButton(driverController, Button.kRightStick.value)
    // 		.onTrue(new /*Command*/);
        
    // new JoystickButton(driverController, Button.kLeftStick.value)
    // 		.onTrue(new /*Command*/);
      
    // new DPadButton(driverController, DPadButton.Direction.UP)
    // 		.onTrue(new /*Command*/);

    // new DPadButton(driverController, DPadButton.Direction.DOWN)
    // 		.whileTrue(new /*Command*/);


    //OPERATOR//
        
    // new JoystickButton(operatorController, Button.kA.value)
    // 		.onTrue(new /*Command*/)
    // 		.whenReleased(new /*Command*/);

    

    // new JoystickButton(operatorController, Button.kX.value)
    // 		.onTrue(new /*Command*/)
    // 		.whenReleased(new /*Command*/);

    // new JoystickButton(driverController, Button.kY.value)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(operatorController, Button.kStart.value)
    // 		.onTrue(new /*Command*/);

    // new JoystickButton(operatorController, Button.kBack.value)
    // 		.whileTrue(new /*Command*/);

    // new JoystickButton(operatorController, Button.kLeftBumper.value)
    // 		.onTrue(new /*Command*/);
    
    //testing button
    new JoystickButton(operatorController, Button.kRightBumper.value)
    		.onTrue(new ShootSpeakerBumperShot(m_shooter));
    
    //testing button
    new JoystickButton(operatorController, Button.kA.value)
    		.onTrue(new armToBumperShotAngle(m_shooter));

    //testing button
    new JoystickButton(operatorController, Button.kB.value)
    		.onTrue(new armToFartherShotAngle(m_shooter));

    //testing button
    new JoystickButton(operatorController, Button.kY.value)
    		.onTrue(new armToPodiumShotAngle(m_shooter));  
    
    //testing button
    new JoystickButton(operatorController, Button.kX.value)
    		.onTrue(new ShooterOff(m_shooter));
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