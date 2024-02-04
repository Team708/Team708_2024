// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSpeakerBumperShot extends Command {
  private Shooter m_shooter;

  /** Creates a new Shoot. */
  //we need a feeder system created and implemented


  public ShootSpeakerBumperShot(Shooter shooter) {
    this.m_shooter = shooter;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_shooter);
  }

  @Override
  public void initialize(){
    
  }

  public void execute(){
    m_shooter.setShooterSpeedSpeaker(Constants.ShooterConstants.kShooterHighCloseMPS);
  }
  
  @Override
  public boolean isFinished(){
    return m_shooter.isShooterSpeakerAtSpeed();
  }
}
