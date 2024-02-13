// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.List;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveToAmp extends Command {
  Drivetrain m_drive;
  PathConstraints trajectoryConstraints;

  /** Creates a new SimplerDriveToAmp. */
  public DriveToAmp(Drivetrain dr) {
    m_drive = dr;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_drive.driveToPoint(Constants.DriveConstants.kRobotToAmp);
    // Command pathfindingCommand = new PathfindHolonomic(
    //   Constants.DriveConstants.kRobotToAmp, 
    //   Constants.DriveConstants.kTrajectoryConstraints, 
    //   0.0,
    //   m_drive.getPose(),
    //   m_drive.getChassisSpeed(),       
    //   getChassisSpeed(),
    //   Constants.DriveConstants.pathFollowingConfig,
    //   0.0,
    //   m_drive);
    m_drive.driveToPoint(DriveConstants.kRobotToAmp);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // m_drive.runPathfindingCommand();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   // Pose2d currTolerance = m_drive.getPose().relativeTo(desiredLocation);
    
  //   // if(Math.abs(currTolerance.getX()) <= Constants.DriveConstants.kPositionTolerance.getX() && 
  //   // Math.abs(currTolerance.getY()) <= Constants.DriveConstants.kPositionTolerance.getY() && 
  //   // Math.abs(currTolerance.getRotation().getDegrees()) <= Constants.DriveConstants.kPositionTolerance.getRotation().getDegrees()){
  //   //     return true;
  //   // }
  //   return false;
  // }
}
