// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
//Library imports
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//Command imports
import frc.robot.commands.drive.EnableAutoTargetSpeaker;
import frc.robot.commands.shooter.SetShooterSpeedSpeaker;
import frc.robot.commands.Feeder.FeederAutomatic;
import frc.robot.commands.PivotArm.EnableArmAutoAim;
import frc.robot.commands.intake.IntakeAllAutomatic;
//Subsystem imports
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.PivotArm;
import frc.robot.subsystems.Shooter;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSpeakerPCG extends ParallelRaceGroup {
  /** Creates a new ShootSpeakerPCG. */
  public ShootSpeakerPCG(Drivetrain m_drive, Intake m_intake, Feeder m_feeder, PivotArm m_pivotArm, Shooter m_shooter) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    // super(new EnableAutoTargetSpeaker(m_drive));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ParallelCommandGroup(
        new EnableAutoTargetSpeaker(m_drive),
        new IntakeAllAutomatic(m_intake),
        new FeederAutomatic(m_feeder),
        new EnableArmAutoAim(m_pivotArm),
        new SetShooterSpeedSpeaker(m_shooter)
      // )
    );
  }
}
