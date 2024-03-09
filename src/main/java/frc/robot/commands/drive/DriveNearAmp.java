// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveNearAmp extends Command {
  /** Creates a new DriveNearAmp. */
  private Drivetrain m_drive;
  private PathConstraints trajectoryConstraints;

  private Command cmd;

  public DriveNearAmp(Drivetrain dr) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = dr;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectoryConstraints = DriveConstants.ktrajectoryConstraints;

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(m_drive.getPose(), DriveConstants.kRobotToAmp.transformBy(new Transform2d(0,0,new Rotation2d(Math.PI))));
    PathPlannerPath path1 = new PathPlannerPath(
      bezierPoints, 
      trajectoryConstraints,  
      new GoalEndState(0.0, DriveConstants.kRobotToAmp.getRotation())
    );

    cmd = AutoBuilder.followPath(path1);
    
    cmd.schedule();
    cmd.cancel();
    cmd.end(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cmd.cancel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmd.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(OI.getDriverRightX()) >= ControllerConstants.kDriverDisableAutoTargeting ||
            Math.abs(OI.getDriverLeftX()) >= ControllerConstants.kDriverDisableAutoTargeting  ||
            Math.abs(OI.getDriverLeftY()) >= ControllerConstants.kDriverDisableAutoTargeting);
  }
}
