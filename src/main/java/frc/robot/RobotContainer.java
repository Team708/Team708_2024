// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.FMSConstants;
import frc.robot.commands.auto.doNothingCommand;
import frc.robot.commands.DriveByController;
// import frc.robot.commands.OperateByController; //TODO uncomment if using Operator Controller

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems. Initialize subsystems here.
	private final Drivetrain m_drive = new Drivetrain();
	public static Intake m_intakeSubsystem = new Intake();
	private final Shooter m_shooter = new Shooter();

	// Initialize controllers
	private final DriveByController m_driveByController =  new DriveByController(m_drive);
	// private final OperateByController m_operateByController
	// = new OperateByController(m_shooter); 

	// Autonomous Option
	private final Command doNothin = new WaitCommand(5);

	public static final SendableChooser<Command> m_chooser = new SendableChooser<>();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
		configureAutoChooser();

		m_drive.setDefaultCommand(m_driveByController);
		m_intakeSubsystem.setDefaultCommand(m_driveByController);

		m_drive.resetOdometry(new Pose2d()); //TODO need to test. Pigeon position does not reset on hardware
	}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new POVButton(OI.driverController, 0)
    //     .onTrue(new InstantCommand(() -> m_drive.resetOdometry(new Rotation2d(0.0))));  //JNP

    OI.configureButtonBindings(m_drive,m_shooter);
  }

	private void configureAutoChooser(){
		m_chooser.addOption("Do Nothing",     doNothin);
		m_chooser.setDefaultOption("Do Nothing", doNothin);
		SmartDashboard.putData(m_chooser);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// this is where we run the initialization -see instantcommand: executes a
		// single action on initialization, and then ends immediately (on page
		// convenience features)
		// or do we put it in the autonomousInit in robot.java?
		return m_chooser.getSelected();
	}

	public Drivetrain getDrivetrain() {
		return m_drive;
	}

	public Intake getIntake() {
		return m_intakeSubsystem;
	}

	public void simulationInit(){
	//   m_elevator.simulationInit();
	}

	/** This function is called periodically whilst in simulation. */
	public void simulationPeriodic() {
	//   m_elevator.simulationPeriodic();
	}

	public void sendToDashboard() {
		m_drive.sendToDashboard();
		m_intakeSubsystem.sendToDashboard();
		// m_shooter.sendToDashboard();
		// m_climber.sendToDashboard();
		// m_intakeFeeder.sendToDashboard();
		// m_limelight.sendToDashboard();
		// m_candleSystem.sendToDashboard();
	}

}
