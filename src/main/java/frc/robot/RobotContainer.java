// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.DriveByController;
import frc.robot.commands.OperateByController; //TODO uncomment if using Operator Controller
import frc.robot.commands.ShootSpeakerPCG;
import frc.robot.commands.ShootSpeakerOneShot;

import frc.robot.commands.shooter.SetShooterSpeedSpeaker;
import frc.robot.commands.Feeder.FeederForward;
import frc.robot.commands.AllSystemsOff;
import frc.robot.commands.ClimbByController;
import frc.robot.commands.groups.IntakeNote;
import frc.robot.commands.drive.DriveToAmp;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionProcessor;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utilities.PidHelper;
import edu.wpi.first.wpilibj2.command.Command;

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
	private final Shooter m_shooter = new Shooter();
	private final VisionProcessor m_vision = new VisionProcessor(m_drive);
	private final PivotArm m_pivotArm = new PivotArm(m_drive);
	private final Feeder m_feeder = new Feeder(m_drive, m_pivotArm, m_shooter);
	private final Intake m_intake = new Intake(m_feeder);
	private final Climber m_climber = new Climber();
	
	
	// Initialize controllers
	private final DriveByController m_driveByController =  new DriveByController(m_drive);
	private final OperateByController m_operateByController = new OperateByController(m_pivotArm);
	private final ClimbByController m_climbByController = new ClimbByController(m_climber); 

	private final AllSystemsOff m_AllSystemsOff = new AllSystemsOff(m_intake, m_feeder, m_shooter);
	// Autonomous Option

	// public static final SendableChooser<Command> m_chooser = new SendableChooser<>();
    public final SendableChooser<Command> autoChooser;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();
		
		
		NamedCommands.registerCommand("ShootSpeakerPCG", new ShootSpeakerPCG(m_drive, m_intake, m_feeder, m_pivotArm, m_shooter));
		NamedCommands.registerCommand("SetShooterSpeedSpeaker", new SetShooterSpeedSpeaker(m_shooter));
		NamedCommands.registerCommand("FeederFoward", new FeederForward(m_feeder));
		NamedCommands.registerCommand("IntakeNote", new IntakeNote(m_intake, m_feeder));
		NamedCommands.registerCommand("DriveToAmp", new DriveToAmp(m_drive));
		NamedCommands.registerCommand("ShootSpeakerOneShot", new ShootSpeakerOneShot(m_drive, m_intake, m_feeder, m_pivotArm, m_shooter));
		// NamedCommands.registerCommand("ResetDrive", new ResetDrive(m_drive, ));

	    // configureAutoChooser();
		// Build an auto chooser. This will use Commands.none() as the default option.
		autoChooser = AutoBuilder.buildAutoChooser();

		// getAutonomousCommand();
		m_drive.setDefaultCommand(m_driveByController);
		m_pivotArm.setDefaultCommand(m_operateByController);
		m_climber.setDefaultCommand(m_climbByController);

		SmartDashboard.putData("Auto Chooser", autoChooser);
		
		m_drive.setPose(new Pose2d()); //TODO need to test. Pigeon position does not reset on hardware
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

    OI.configureButtonBindings(m_drive,m_intake,m_feeder,m_shooter,m_pivotArm);
  }

	// private void configureAutoChooser(){
	// 	m_chooser.addOption("Do Nothing",     doNothin);
	// 	m_chooser.setDefaultOption("Do Nothing", doNothin);
	// 	SmartDashboard.putData(m_chooser);
	// }

	public Drivetrain getDrivetrain() {
		return m_drive;
	}

	// public Intake getIntake() {
	// 	return m_intake;
	// }

	public Shooter getShooter() {
		return m_shooter;
	}

	public PivotArm getPivotArm() {
		return m_pivotArm;
	}

	public VisionProcessor getVision() {
		return m_vision;
	}
	
	public void simulationInit(){
	  m_intake.simulationInit();
	}

	/** This function is called periodically whilst in simulation. */
	public void simulationPeriodic() {
	  m_intake.simulationPeriodic();
	}

	public Command allSystemsOff() {
		return m_AllSystemsOff;
	}
	public void sendToDashboard() {
		// PidHelper.getInstance().update(); //TODO Uncomment to tune PID
		m_drive.sendToDashboard();
		m_intake.sendToDashboard();
		m_feeder.sendToDashboard();
		m_shooter.sendToDashboard();
		m_pivotArm.sendToDashboard();
	}
}