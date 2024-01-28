package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.*;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utilities.MathUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

  /**
   * Implements a DriveByController command which extends the Command class
   */
public class DriveByController extends Command {
  private final Drivetrain m_robotDrive;

  // double autoAngle = 0.0;
  // double lastSpeed = 0.0;
  // double lastTime = Timer.getFPGATimestamp();
  // PIDController controller = new PIDController(0.06, 0.0001, 0.0025);//, //new Constraints(300000, 150000));
  // double desiredRot;

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
  public void initialize(){
  }

  /**
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {
    // double maxLinear = DriveConstants.kMaxSpeedMetersPerSec;
    // double desiredX = -inputTransform(OI.getDriverLeftY())*maxLinear;
    // double desiredY = -inputTransform(OI.getDriverLeftX())*maxLinear;
    // double maxLinear = DriveConstants.kMaxSpeedMetersPerSec;
    // Translation2d desiredTranslation = m_robotDrive.getDriverXAndY(maxLinear);
    // double desiredMag = desiredTranslation.getDistance(new Translation2d());
    m_robotDrive.makeRobotDrive();
    //double desiredRot = m_robotDrive.findAutoRotate(controller, m_robotDrive.getDriverRot());
  
    // if(Math.abs(desiredRot) > 0.08){
    //   autoRotEnabled = false;
    // }
    // if(autoRotEnabled){
    //     double dx = Constants.DriveConstants.kPoseSpeakerBumperBottom.getX() - m_robotDrive.getPose().getX();
    //     double dy = Constants.DriveConstants.kPoseSpeakerBumperBottom.getY() - m_robotDrive.getPose().getY();
    //     Rotation2d robotToTarget = new Rotation2d(dx, dy);
    //     desiredRot = controller.calculate(m_robotDrive.getPose().getRotation().getDegrees(), robotToTarget.getDegrees());
    //     SmartDashboard.putNumber("autoAngle", robotToTarget.getDegrees());
    //     SmartDashboard.putBoolean("IsTargetingOn", autoRotEnabled);
    //     // if(controller.atSetpoint()){
    //     //   autoRotEnabled = false;
    //     // }
    //}

    // System.out.println(manualRotEnabled);
    
    // if(desiredMag >= maxLinear){
    //   desiredTranslation.times(maxLinear/desiredMag);
    // }
    // m_robotDrive.drive(m_robotDrive.getDriverXAndY().getX(), 
    //                    m_robotDrive.getDriverXAndY().getY(),
    //                    desiredRot,
    //                    m_robotDrive.getFieldOrient(),
    //                    true);
  }

  @Override
  public void end(boolean interrupted){

  }

  /**
   * This function takes the user input from the controller analog sticks, applys a deadband and then quadratically
   * transforms the input so that it is easier for the user to drive, this is especially important on high torque motors 
   * such as the NEOs or Falcons as it makes it more intuitive and easier to make small corrections
   * @param input is the input value from the controller axis, should be a value between -1.0 and 1.0
   * @return the transformed input value
   */
  // private double inputTransform(double input){
  //   //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
  //   return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
  // }


}
