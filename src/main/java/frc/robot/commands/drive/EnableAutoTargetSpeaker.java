// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.drive.Drivetrain;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EnableAutoTargetSpeaker extends Command {

  Drivetrain m_drive;

  public EnableAutoTargetSpeaker(Drivetrain drive) {
    m_drive = drive;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setAutoRot(true);
    SmartDashboard.putString("Command", this.getName() + ": Init");
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(OI.getDriverRightX()) >= ControllerConstants.kDriverDisableAutoTargeting);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setAutoRot(false);
    SmartDashboard.putString("Command", this.getName() + ": End");
  }
}
