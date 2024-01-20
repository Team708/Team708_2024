// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// import frc.robot.commands.elevator.ElevatorToNode;
// import frc.robot.commands.intake.*;
// import frc.robot.commands.groups.RaiseElevWhenPiece;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drivetrain;
// import frc.robot.subsystems.intake.Intake;

public class DogLeg extends SequentialCommandGroup {

  public DogLeg(Drivetrain dr, double maxSpeed) {
    // PathPlannerPath path1 = PathPlannerPath.fromPathFile("DogLeg");
    // AutoBuilder.followPath(path1);
  }
    
}