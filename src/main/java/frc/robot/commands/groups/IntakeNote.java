// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Feeder.FeedNoteForStorage;
import frc.robot.commands.intake.IntakeAllAutomatic;
import frc.robot.commands.shooter.ShooterReverse;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class IntakeNote extends ParallelDeadlineGroup {
  /** Creates a new IntakeNote. */

  public IntakeNote(Intake m_intake, Feeder m_feeder) {

    super(new FeedNoteForStorage(m_feeder));
    addCommands(new IntakeAllAutomatic(m_intake));
  }
}
