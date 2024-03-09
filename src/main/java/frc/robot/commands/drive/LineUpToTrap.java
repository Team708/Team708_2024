// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utilities.FMSData;                                                   
import frc.robot.utilities.Limelight;

public class LineUpToTrap extends Command {
  /** Creates a new LineUpToTrap. */
  private Drivetrain m_Drivetrain;
  private Pose2d robotPose;
  private List<Pose2d> trapPoses = new ArrayList<>();
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


  public LineUpToTrap(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Drivetrain = drivetrain;
    
  
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rotController.setTolerance(Units.degreesToRadians(3));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_Drivetrain);
    robotPose = m_Drivetrain.getPose();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(FMSData.allianceIsRed())
    // {
    //   trapPoses.add(GeometryUtil.flipFieldPose(DriveConstants.kBlueStageBackTrap));
    //   trapPoses.add(GeometryUtil.flipFieldPose(DriveConstants.kBlueStageLeftTrap));
    //   trapPoses.add(GeometryUtil.flipFieldPose(DriveConstants.kBlueStageRightTrap));
    // }
    // else{
    //   trapPoses.add(DriveConstants.kBlueStageBackTrap);
    //   trapPoses.add(DriveConstants.kBlueStageLeftTrap);
    //   trapPoses.add(DriveConstants.kBlueStageRightTrap);
    // }
    // // targetPose = robotPose.nearest(trapPoses);
    targetPose = DriveConstants.kRobotToAmp;
    // m_Drivetrain.driveToTrapPose(targetPose);

    rotController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = m_Drivetrain.getPose();

    // Drive
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    rotController.setGoal(targetPose.getRotation().getRadians());

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
