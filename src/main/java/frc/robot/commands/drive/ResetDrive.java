package frc.robot.commands.drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drivetrain;

public class ResetDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final Rotation2d m_orientation;

  public ResetDrive(Drivetrain drivetrain, Rotation2d orientation) {
    m_drivetrain = drivetrain;
    m_orientation = orientation;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setPose(new Pose2d(0.0,0.0,m_orientation));
    OI.driverController.setRumble(RumbleType.kBothRumble, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}