// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PivotArm.armToAmpShotAngle;
import frc.robot.commands.shooter.SetShooterSpeedAmp;
import frc.robot.commands.shooter.SetShooterSpeedSpeaker;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.Feeder.FeedNoteToShoot;
import frc.robot.commands.drive.DriveToAmp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.drive.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAmpSequence extends SequentialCommandGroup {
  /** Creates a new ShootSpeakerBumperShotSCG. 
   * @param PivotArm
   * @param Shooter
   * @param Feeder
   * @param Drivetrain */

  public ShootAmpSequence(Drivetrain m_drive, Feeder m_feeder, Shooter m_shooter, PivotArm m_PivotArm) {
    addCommands(
      new ParallelCommandGroup(
        //new DriveToAmp(m_drive),
        new armToAmpShotAngle(m_PivotArm), 
        new SequentialCommandGroup(
          new SetShooterSpeedAmp(m_shooter),
          new FeedNoteToShoot(m_feeder, m_PivotArm).withTimeout(2.0),
          new ShooterOff(m_shooter)
        ) 
      )
      //  //keep aiming while shooting
      // new ParallelCommandGroup(
      //   new EnableAutoTargetSpeaker(m_drive),
      //   new EnableArmAutoAim(m_PivotArm, m_drive), 
      //   new SetShooterSpeedBumperShot(m_shooter),
      //   new FeedNoteToShoot(m_feeder)
      // ),
    );
  }
}