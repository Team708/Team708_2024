// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeAllIn extends Command{
    private final Intake m_intake;

    /** Creates a new IntakeOut. */
    public IntakeAllIn(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // You can perform any initialization here.
        m_intake.intakeAll();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Call the method from the Intake subsystem to intake all
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // You can perform any actions when the command ends or is interrupted.
        // For example, stop the intake subsystem.
        // m_intake.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Return true or false based on your condition
        return false;
    }
}
