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
public class ShootSpeaker extends Command {

  private final Shooter m_shooterMotorTop;

  /** Creates a new Shoot. */
  //we need a feeder system created and implemented


  public ShootSpeaker(Shooter shootersubsystem) {
    m_shooterMotorTop = shootersubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(m_shooterMotorTop);
  }

  @Override
  public void initialize(){
    m_shooterMotorTop.shooterVelocitySpeaker(Constants.ShooterConstants.kShooterHighCloseMPS);
  }
  
  @Override
  public boolean isFinished(){
    return m_shooterMotorTop.isShooterAtSpeed();
  }
}
