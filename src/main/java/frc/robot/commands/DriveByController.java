package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

  /**
   * Implements a DriveByController command which extends the Command class
   */
public class DriveByController extends Command {
  private final Drivetrain m_robotDrive;

  /**
   * Contructs a DriveByController object which applys the driver inputs from the
   * controller to the swerve drivetrain
   * 
   * @param drive      is the swerve drivetrain object which should be created in
   *                   the RobotContainer class
   * @param controller is the user input controller object for controlling the
   *                   drivetrain
   */
  public DriveByController(Drivetrain drive) {
    m_robotDrive = drive; // Set the private member to the input drivetrain
    Constants.DriveConstants.kAutoRotatePID.enableContinuousInput(-180, 180);
    Constants.DriveConstants.kAutoRotatePID.setTolerance(0.5, 10); //Degrees?
    addRequirements(m_robotDrive); // Because this will be used as a default command, add the subsystem which will
                                   // use this as the default
  }

  @Override
  public void initialize() {
  }

  /**
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {
    m_robotDrive.driveByController();
  }

  @Override
  public void end(boolean interrupted) {
    
  }

}
