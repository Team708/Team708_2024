// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Feeder.FeederOff;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class AllSystemsOff extends ParallelCommandGroup {

  public AllSystemsOff(Intake m_intake, Feeder m_feeder, Shooter m_shooter) {

    addCommands(
      new ShooterOff(m_shooter),
      new IntakeOff(m_intake),
      new FeederOff(m_feeder)
    );
  }
}
