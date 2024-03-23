// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeederOneShot extends Command {
  private Feeder m_feeder;
  /** Creates a new FeederAutomatic. */
  public FeederOneShot(Feeder feeder) {
    m_feeder = feeder;
    addRequirements(m_feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.feederAutomatic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
  }

  // Returns true when the command should end. 
  @Override
  public boolean isFinished() {
    return m_feeder.isEmpty();
  }
}
