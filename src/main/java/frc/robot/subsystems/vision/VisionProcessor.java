package frc.robot.subsystems.vision;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionProcessorConstants;
import frc.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionProcessor extends SubsystemBase {

  public static Drivetrain drivetrain;
  // private static Intake intake;
  private Limelight limelight;
  private boolean led = false;
  // private boolean isAtY = false;
  public double robotSide;
  public boolean targetFound = false;
  // private boolean isCentered = false;
  // private boolean isAtArea = false;

  // Required Network Table Data
  private boolean seesTarget; // Get from Network Table
  private double tv;
  // private double yAngle;
  // private double difference;

  // private double xAngle; //Get from Network Table
  // private double area;
  // private double rotate = 0.0;
  // private double move = 0.0;
  
  public VisionProcessor(Drivetrain m_drive) {
    drivetrain = m_drive;
    limelight = new Limelight();
    setName("Vision Processor");
  }

  @Override
  public void periodic() {
    drivetrain.m_PoseEstimator.addVisionMeasurement(limelight.getBotPose2d(VisionConstants.klimelightName),(Timer.getFPGATimestamp()-(cl/1000)-(tl/1000)));
  }

  public boolean seesTarget() {
    return Limelight.getTV(VisionConstants.klimelightName);
  }

  public void toggleLEDMode() {
    led = !led;
    if (led)
      setNTInfo("ledMode", VisionProcessorConstants.kVisionLedOn);
    else
      setNTInfo("ledMode", VisionProcessorConstants.kVisionLedOff);
  }

  public double getRotate() {
    // return getNTInfo("tx");
    return -getNTInfo("ty");
  }

  // public void findTarget() { //drivetrain vision processing
  // double angle = getRotate();
  // if (Math.abs(angle) > 1.0) {
  // Swerve.getInstance().rotateDegreesfromPosition(angle);
  // targetFound = false;
  // }
  // else
  // targetFound = true;
  // }

  public double getDistance() {
    if (seesTarget())
      // return 74/Math.tan(Math.PI*((getNTInfo("ty")+20)/180));
      return 72.5 / Math.tan(Math.PI * ((getNTInfo("tx") + 40) / 180)); // target 94" - camera height 21.5"
                                        // ty = camera angle + Ty
    else
      return 0;
  }

  public void sendToDashboard() {
    SmartDashboard.putBoolean("Vision Has Target", targetFound);
    SmartDashboard.putNumber("Vision_Distance ", getDistance());
    SmartDashboard.putNumber("Vision tx", getRotate());
  }

}
