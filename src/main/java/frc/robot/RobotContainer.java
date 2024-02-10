// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;

// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.auto.FiveBall;
import frc.robot.commands.auto.DriveStraight;

import frc.robot.commands.DriveByController;
// import frc.robot.commands.OperateByController; //TODO uncomment if using Operator Controller

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import frc.robot.commands.OperateByController; //TODO uncomment if using Operator Controller

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.vision.VisionProcessor;
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
	private final VisionProcessor m_vision = new VisionProcessor(m_drive);
	private final Intake m_intake = new Intake();

	// Initialize controllers
	private final DriveByController m_driveByController =  new DriveByController(m_drive);
	// private final OperateByController m_operateByController
	// = new OperateByController(/*Subsystem*/); // TODO Add operator controller

	// Autonomous Option
	private final Command doNothin = new WaitCommand(5);
	private final Command FiveBall = new FiveBall(m_drive, 8);
	private final Command DriveStraight = new DriveStraight(m_drive, 8);

	// public static final SendableChooser<Command> m_chooser = new SendableChooser<>();
    private final SendableChooser<Command> autoChooser;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
	
		// configureAutoChooser();
		// Build an auto chooser. This will use Commands.none() as the default option.
		autoChooser = AutoBuilder.buildAutoChooser();
		// autoChooser.addOption("Five Ball", FiveBall);
		// autoChooser.addOption("Drive Straight", DriveStraight);
		
		// getAutonomousCommand();
		m_drive.setDefaultCommand(m_driveByController);

		SmartDashboard.putData("Auto Chooser", autoChooser);
		
		m_drive.resetOdometry(new Pose2d()); //TODO need to test. Pigeon position does not reset on hardware
	}

    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("DriveStraight");
		return autoChooser.getSelected();
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

    OI.configureButtonBindings(m_drive, m_intake);
  }

	// private void configureAutoChooser(){
	// 	m_chooser.addOption("Five Ball",  FiveBall);
	// 	m_chooser.addOption("Do Nothing",     doNothin);
	// 	m_chooser.setDefaultOption("Do Nothing", doNothin);
	// 	SmartDashboard.putData(m_chooser);
	// }

	public Drivetrain getDrivetrain() {
		return m_drive;
	}

	public Intake getIntake() {
		return m_intake;
	}

	public void simulationInit(){
	  //m_intake.simulationInit();
	}

	/** This function is called periodically whilst in simulation. */
	public void simulationPeriodic() {
	  //m_intake.simulationPeriodic();
	}

	public void sendToDashboard() {
		m_drive.sendToDashboard();
		m_intake.sendToDashboard();
		// m_shooter.sendToDashboard();
		// m_climber.sendToDashboard();
		// m_limelight.sendToDashboard();
		// m_candleSystem.sendToDashboard();
	}
}