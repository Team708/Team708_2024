// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotArm.armToBumperShotAngle;
import frc.robot.commands.shooter.SetShooterSpeedBumperShot;
import frc.robot.commands.Feeder.FeedNoteToShoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Feeder;




// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSpeakerBumperShotSCG extends SequentialCommandGroup {
  /** Creates a new ShootSpeakerBumperShotSCG. 
   * @param PivotArm 
   * @param m_shooter
   * @param m_feeder */

  public ShootSpeakerBumperShotSCG(PivotArm PivotArm, Shooter m_shooter, Feeder m_feeder) {
    
    //

    
    addCommands(
      
      new ParallelCommandGroup(
          new armToBumperShotAngle(PivotArm), 
          new SetShooterSpeedBumperShot(m_shooter)
          //add autoturning command
      ),
      new FeedNoteToShoot(m_feeder) //keep aiming while shooting, 
    );
  }
}