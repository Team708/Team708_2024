// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class TurnToTarget extends Command {
  /** Creates a new TurnToTarget. */
  private Drivetrain m_drive;
  private ChassisSpeeds blankChassisSpeeds;
  public TurnToTarget(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setAutoRot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    blankChassisSpeeds = new ChassisSpeeds(0,0,0);
    m_drive.driveRobotRelative(blankChassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setAutoRot(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
