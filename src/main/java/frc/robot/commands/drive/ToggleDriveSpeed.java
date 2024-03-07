package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class ToggleDriveSpeed extends Command{
  private final Drivetrain m_drivetrain;

  public ToggleDriveSpeed(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_drivetrain.getSpeedSlow())
       m_drivetrain.setSpeedFast();
       else
       m_drivetrain.setSpeedSlow();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
    
}
