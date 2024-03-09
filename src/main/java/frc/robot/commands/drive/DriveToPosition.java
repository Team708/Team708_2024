// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToPosition extends Command {
  /** Creates a new DriveToPosition. */
  private Drivetrain m_Drivetrain;
  private Pose2d robotPose;
  private Pose2d targetPose;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(
      DriveConstants.kMaxSpeedMetersPerSec, DriveConstants.kMaxAccelMetersPerSecSquared);
  private TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(
      DriveConstants.kMaxSpeedMetersPerSec, DriveConstants.kMaxAccelMetersPerSecSquared);
  private TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(
      DriveConstants.kMaxAngularSpeedRadPerSec, DriveConstants.kMaxAngularAccel);

  private ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, xConstraints);
  private ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, yConstraints);
  private ProfiledPIDController rotController = new ProfiledPIDController(2, 0, 0, rotConstraints);

  public DriveToPosition(Drivetrain drivetrain, Pose2d target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Drivetrain = drivetrain;
    addRequirements(m_Drivetrain);
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rotController.setTolerance(Units.degreesToRadians(3));
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    targetPose = target;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    // Drive
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    rotController.setGoal(targetPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = m_Drivetrain.getPose();

    // Drive to the target
    xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    rotSpeed = rotController.calculate(robotPose.getRotation().getRadians());
    if (rotController.atGoal()) {
      rotSpeed = 0;
    }

    m_Drivetrain.drive(xSpeed, ySpeed, rotSpeed, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(OI.getDriverRightX()) >= ControllerConstants.kDriverDisableAutoTargeting ||
            Math.abs(OI.getDriverLeftX()) >= ControllerConstants.kDriverDisableAutoTargeting  ||
            Math.abs(OI.getDriverLeftY()) >= ControllerConstants.kDriverDisableAutoTargeting);
  }
}
