// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterOff extends InstantCommand {
  private Shooter m_shooterMotorTop;
  public ShooterOff(Shooter shootersubsystem) {
    m_shooterMotorTop = shootersubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterMotorTop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterMotorTop.stopShooter();
  }
}
