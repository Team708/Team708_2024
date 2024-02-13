// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DisableAutoTargetSpeaker extends InstantCommand {

  Drivetrain dr;

  public DisableAutoTargetSpeaker(Drivetrain m_drive) {
    this.dr = m_drive;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    dr.disableAutoRot();
  }


}
