// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// // import frc.robot.utilities.AutoFromPathPlanner;
// // import frc.robot.commands.elevator.ElevatorToNode;
// // import frc.robot.commands.intake.*;
// // import frc.robot.commands.groups.RaiseElevWhenPiece;
// // import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.drive.Drivetrain;
// // import frc.robot.subsystems.intake.Intake;

// public class FiveBall extends SequentialCommandGroup {

//   public FiveBall(Drivetrain dr, double maxSpeed) {
//     // AutoFromPathPlanner path1 = new AutoFromPathPlanner(dr, "FiveBall", maxSpeed, true);
//     // AutoFromPathPlanner path2 = new AutoFromPathPlanner(dr, "LeftSideDriveToPiece", maxSpeed, true);
//     // AutoFromPathPlanner path3 = new AutoFromPathPlanner(dr, "LeftSideDriveToPieceBalance", maxSpeed, true);

//     // addCommands(
//       // new InstantCommand(() -> dr.setPose(path1.getInitialPose())),
//       // new IntakeOn(m_intake),
//       // new WaitCommand(0.2),
//       // new IntakeOut(m_intake).withTimeout(.2),
//       // new WaitCommand(0.2),
//       // new IntakeOff(m_intake),
//       // path1//,

//       // new WaitCommand(.2),
//       // path2,
//       // new IntakeOut(m_intake, m_candle).withTimeout(.2),
//       // new WaitCommand(0.2),
//       // new IntakeOff(m_intake),
//       // new WaitCommand(0.2),
//       // path3,

//       // new WaitCommand(0.2),
//       // new InstantCommand(() -> dr.setPose(path1.getInitialPose()))
//       // );
//   }
    
// }