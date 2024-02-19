// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.PivotArm;

public class FeedNoteToShoot extends Command {
  private Feeder m_feeder;
  private PivotArm m_pivotArm;

  public FeedNoteToShoot(Feeder feeder, PivotArm pivotArm) {
    m_feeder = feeder;
    m_pivotArm = pivotArm;

    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(m_pivotArm.isArmAtPosition()) {
      m_feeder.runForward(1.0);
    //}
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop();
  }
  @Override
  public boolean isFinished() {
    return m_feeder.isEmpty();
  }

}
