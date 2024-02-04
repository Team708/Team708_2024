// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ArmConstants;

public class armToPositionSetPoint1 extends Command {
  /** Creates a new moveArm. */
  Shooter m_shooter;
  

  public armToPositionSetPoint1(Shooter m_shooter) {
    this.m_shooter = m_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_shooter.setAngle(ArmConstants.kPodiumShotAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_shooter.findDisplacement(ArmConstants.kPodiumShotAngle)) < ArmConstants.kThresholdArm);
  }
}
