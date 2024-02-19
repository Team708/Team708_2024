// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class FeedNoteForStorage extends Command {
  Feeder m_feeder;

  public FeedNoteForStorage(Feeder feeder) {
    m_feeder = feeder;
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.feedNotesToStow(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.feedNotesToStow(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      // Return true or false based on your condition
      return m_feeder.hasNote();
  }
}
