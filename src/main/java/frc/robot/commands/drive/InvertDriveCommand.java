package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class InvertDriveCommand extends Command {
	private final Drivetrain m_DriveSubsystem;

	public InvertDriveCommand(Drivetrain drivetrain) {
		m_DriveSubsystem = drivetrain;
		addRequirements(m_DriveSubsystem);
	}

	@Override
	public void initialize() {
		m_DriveSubsystem.invertDrive();;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
