// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;



public class IntakeAllOut extends Command {
  private final Intake m_intake;
  private final Feeder m_feeder;

  /** Creates a new IntakeOut. */
  public IntakeAllOut(Intake intake, Feeder feeder) {
    m_intake = intake;
    m_feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_intake);
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakeOff();
    m_feeder.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakeReverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
