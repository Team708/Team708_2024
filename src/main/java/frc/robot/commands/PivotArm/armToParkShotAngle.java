// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotArm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotArm;
import frc.robot.Constants.ArmConstants;

public class armToParkShotAngle extends Command {
  /** Creates a new moveArm. */
  PivotArm m_PivotArm;
  
  public armToParkShotAngle(PivotArm PivotArm) {
    m_PivotArm = PivotArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_PivotArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("Command", this.getName() + ": Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_PivotArm.setArmAngle(ArmConstants.kParkAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Command", this.getName() + ": End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_PivotArm.isArmAtPosition(ArmConstants.kParkAngle));
  }
}
