// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PivotArm.armToTrapShotAngle;
import frc.robot.commands.shooter.SetShooterSpeedTrap;
import frc.robot.commands.shooter.SetShooterSpeedSpeaker;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.Feeder.FeedToShoot;
import frc.robot.commands.Feeder.FeederForward;
import frc.robot.commands.drive.DriveToAmp;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.drive.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootTrapSequence extends SequentialCommandGroup {
  /** Creates a new ShootSpeakerBumperShotSCG. 
   * @param PivotArm
   * @param Shooter
   * @param Feeder
   * @param Drivetrain */

  public ShootTrapSequence(Drivetrain m_drive, Feeder m_feeder, Shooter m_shooter, PivotArm m_PivotArm) {
    addCommands(
      new SequentialCommandGroup(
        new armToTrapShotAngle(m_PivotArm), 
        new SetShooterSpeedTrap(m_shooter),  
        new FeedToShoot(m_feeder).withTimeout(5.0),
        new ShooterOff(m_shooter)
        ) 
      );
  }
}