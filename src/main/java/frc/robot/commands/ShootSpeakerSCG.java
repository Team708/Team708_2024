// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PivotArm.EnableArmAutoAim;
import frc.robot.commands.shooter.SetShooterSpeedSpeaker;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.Feeder.FeedNoteToShoot;
import frc.robot.commands.drive.EnableAutoTargetSpeaker;
import frc.robot.commands.groups.IntakeNote;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSpeakerSCG extends ParallelDeadlineGroup {
  /** Creates a new Test_ShootSpeakerSCG. */
  public ShootSpeakerSCG(Drivetrain m_drive, Feeder m_feeder, Shooter m_shooter, PivotArm m_PivotArm, Intake m_intake) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new EnableAutoTargetSpeaker(m_drive));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new EnableArmAutoAim(m_PivotArm, m_drive),
        new SequentialCommandGroup(
          new SetShooterSpeedSpeaker(m_shooter),
          new FeedNoteToShoot(m_feeder, m_PivotArm).withTimeout(2.0),
          new ShooterOff(m_shooter)
        )
      )
    );
  }
}
