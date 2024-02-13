// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.drive.Drivetrain;


public class armToBumperShotAngle extends Command {
  /** Creates a new moveArm. */
  PivotArm m_PivotArm;
  Drivetrain m_Drive;
  double angle;
  
  public armToBumperShotAngle(PivotArm pivotArm) {
    m_PivotArm = pivotArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_PivotArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = Math.atan2(2.5, m_Drive.getDistanceToTarget());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
   
    m_PivotArm.setArmAngle(angle);  //replace constant with method from pivotArm subsystem
    //method would take parameter of distance from speaker, then use regression to get arm angle
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_PivotArm.isArmAtPosition(angle));
  }
  
}