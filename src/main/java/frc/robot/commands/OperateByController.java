package frc.robot.commands;

// import frc.robot.OI;
// import frc.robot.Constants.*;
// import frc.robot.utilities.MathUtils;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

  /**
   * Implements a OperateByController command which extends the Command class
   */
public class OperateByController extends Command {

  /**
   * Contructs a OperateByController object which applies the driver inputs from the
   * controller to the shooter
   * 
   * @param shooter      is the shooter object which should be created in
   *                   the RobotContainer class
   * @param controller is the user input controller object for controlling the
   *                   drivetrain
   */
  // public OperateByController(Shooter shooter) {
  //   m_shooter = shooter; // Set the private member to the input shooter
  //   addRequirements(m_shooter); // Because this will be used as a default command, add the subsystem which will
  //                                  // use this as the default
  // }

  @Override
  public void initialize() {
  }

  /**
   * the execute function is overloaded with the function to drive the swerve
   * drivetrain
   */
  @Override
  public void execute() {
    // double maxLinear = shooterConstants.kMaxSpeedMetersPerSecond;
    // double desiredX = inputTransform(OI.getOperatorRightX())*maxLinear;
    // double desiredZ = -inputTransform(OI.getOperatorLeftY())*maxLinear;
    // Translation2d desiredTranslation = new Translation2d(desiredX, desiredZ);
    // double desiredMag = desiredTranslation.getDistance(new Translation2d());

    // if(desiredMag >= maxLinear){
      // desiredTranslation.times(maxLinear/desiredMag);
    // }

    // m_shooter.commandedVelocity(desiredTranslation.getX(), desiredTranslation.getY());
  }

  @Override
  public void end(boolean interrupted) {

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
