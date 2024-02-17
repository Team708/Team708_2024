package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utilities.Limelight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionProcessor extends SubsystemBase {
  double tl, cl;

  public static Drivetrain drivetrain;

  private boolean led = false;
  // private boolean isAtY = false;
  public double robotSide;
  public boolean targetFound = false;
  // private boolean isCentered = false;
  // private boolean isAtArea = false;

  // Required Network Table Data
  // private boolean seesTarget; // Get from Network Table
  // private double tv;
  // private double yAngle;
  // private double difference;

  private Pose2d visionPose;
  // private double xAngle; //Get from Network Table
  // private double area;
  // private double rotate = 0.0;
  // private double move = 0.0;
  
  public VisionProcessor(Drivetrain m_drive) {
    drivetrain = m_drive;
    setName("Vision Processor");

    tl = Limelight.getLatency_Pipeline(VisionConstants.klimelightName);
    cl = Limelight.getLatency_Capture(VisionConstants.klimelightName);
  }


  @Override
  public void periodic() {
    visionPose = Limelight.getBotPose2d_wpiBlue(VisionConstants.klimelightName);
    if(Limelight.getFiducialID(VisionConstants.klimelightName) > 0)
    {
        drivetrain.m_PoseEstimator.addVisionMeasurement(visionPose,(Timer.getFPGATimestamp()-(cl/1000)-(tl/1000)));
    }
    
  }

  public boolean seesTarget() {
    return Limelight.getTV(VisionConstants.klimelightName);
  }

  public void toggleLEDMode() {
    led = !led;
    if (led)
      Limelight.setLEDMode_ForceOn(VisionConstants.klimelightName);
    else
      Limelight.setLEDMode_ForceOff(VisionConstants.klimelightName);
  }

  public double getRotate() {
    return -Limelight.getTY(VisionConstants.klimelightName);
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
      return 72.5 / Math.tan(Math.PI * ((Limelight.getTX(VisionConstants.klimelightName) + 40) / 180)); // target 94" - camera height 21.5"
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
