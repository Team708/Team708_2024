// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotArm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.drive.Drivetrain;


public class EnableArmAutoAim extends Command {
  
  private PivotArm m_pivotArm;
  private double angle;
  
  public EnableArmAutoAim(PivotArm pivotArm) {
    m_pivotArm = pivotArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivotArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putString("Command", this.getName() + ": Init");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // angle = Units.radiansToDegrees(Math.atan2(1.9812, m_drive.getDistanceToTarget()));
    angle = m_pivotArm.findArmAngle();
    m_pivotArm.setArmAngle(angle);
    // SmartDashboard.putNumber("commanded angle", angle);
    //replace constant with method from pivotArm subsystem
    //method would take parameter of distance from speaker, then use regression to get arm angle
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drive.setAutoRot(false);
    // SmartDashboard.putString("Command", this.getName() + ": End");
  }

  // // Returns true when the command should end.
  // @Override
  public boolean isFinished() {
    return false;
  }

}