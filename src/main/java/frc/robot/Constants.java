package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;

 /**
   * Static method containing all constant values for the robot in one location
   */
public final class Constants {
  /**
   * Global constants 
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6;        //Sets a voltage compensation value ideally 12.6V
    public static final int PCMID = 50;
    public static final int PDPID = 51;
    public static final double kLoopTime = 0.020;
  }

  public static final class CurrentLimit{
    public static final int kIntake = 25;

    public static final int kArm = 40;
    public static final int kFeeder = 40;

    public static final int kTranslationAmps = 40;
    public static final int kRotationAmps = 25;
  }

  /**
   * Robot constants 
   */
  public static final class RobotConstants {
    public static final int kRobot = 0; // snowflake = 1, competition = 0
  }

  /**
   * Drivetrain constants 
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 11;   //CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 13;  //CANID of the Translation SparkMAX
    public static final int kRearLeftDriveMotorPort = 15;    //CANID of the Translation SparkMAX
    public static final int kRearRightDriveMotorPort = 17;   //CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 12;   //CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 14;  //CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 16;    //CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 18;   //CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 4;   //Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 5;  //Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 6;    //Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 7;   //Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = -.2189*-1; //-0.5 to 0.5
    public static final double kFrontRightOffset = -.1585*-1; //-0.5 to 0.5
    public static final double kBackLeftOffset = -.5*-1; //-0.5 to 0.5
    public static final double kBackRightOffset = -.1926*-1; //-0.5 to 0.5

    //Drive motor PID is best done on the roboRIO currently as the SparkMAX does not allow for static gain values on the PID controller, 
    //    these are necessary to have high accuracy when moving at extremely low RPMs
    //public static final double[] kFrontLeftTuningVals   =   {0.0120,0.2892,0.25,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kFrontRightTuningVals  =   {0.0092,0.2835,0.25,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackLeftTuningVals    =   {0.0142,0.2901,0.25,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    //public static final double[] kBackRightTuningVals   =   {0.0108,0.2828,0.25,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals   =   {0.001,0.2850,0.2,0};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kFrontRightTuningVals  =   {0.001,0.2850,0.2,1};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackLeftTuningVals    =   {0.001,0.2850,0.2,2};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    public static final double[] kBackRightTuningVals   =   {0.001,0.2850,0.2,3};   //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    //.324 - sideways
    //.414 - longways
    public static final Translation2d kFrontLeftLocation = new Translation2d(0.414,0.324); // +X is forward, +Y is to the right 
    public static final Translation2d kFrontRightLocation = new Translation2d(0.414,-0.324); // +X is forward, +Y is to the right
    public static final Translation2d kBackLeftLocation = new Translation2d(-0.414,0.324); // +X is forward, +Y is to the right
    public static final Translation2d kBackRightLocation = new Translation2d(-0.414,-0.324); // +X is forward, +Y is to the right
    // public static final Translation2d kFrontLeftLocation = new Translation2d(0.33,-0.264); // +X is forward, +Y is to the right 
    // public static final Translation2d kFrontRightLocation = new Translation2d(0.33,0.264); // +X is forward, +Y is to the right
    // public static final Translation2d kBackLeftLocation = new Translation2d(-0.376,-0.264); // +X is forward, +Y is to the right
    // public static final Translation2d kBackRightLocation = new Translation2d(-0.376,0.264); // +X is forward, +Y is to the right
     
    //Because the swerve modules poisition does not change, define a constant SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics 
      = new SwerveDriveKinematics(kFrontLeftLocation,kFrontRightLocation,kBackLeftLocation,kBackRightLocation);

    public static final double kMaxAccelMetersPerSecSquared = 3.75;
    public static final double kMaxSpeedMetersPerSec = 3.5; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeedRadPerSec = 4.0;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = 6.25;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly

    public static final double kInnerDeadband = 0.08; //This value should exceed the maximum value the analog stick may read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; //This value should be lower than the analog stick X or Y reading when aimed at a 45deg angle (Such that X and Y are are maximized simultaneously)
  
    //Minimum allowable rotation command (in radians/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommandRadPerSec = DriveConstants.kMaxAngularSpeedRadPerSec * Math.pow(DriveConstants.kInnerDeadband,2);
    //Minimum allowable tranlsation command (in m/s) assuming user input is squared using quadraticTransform, this value is always positive and should be compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommandMetersPerSec = DriveConstants.kMaxSpeedMetersPerSec * Math.pow(DriveConstants.kInnerDeadband,2);

    public static final double[] kKeepAnglePID = { 0.700, 0, 0 }; //Defines the PID values for the keep angle PID

  }

  /**
   * Swerve Module constants 
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 3.0;          //Units of %power/s, ie 4.0 means it takes 0.25s to reach 100% power from 0%
    private static final double kTranslationGearRatio = 5.6111111; //Overall gear ratio of the swerve module
    private static final double kWheelDiameterMeters = 0.0777*0.98;           //Wheel Diameter in meters, may need to be experimentally determined due to compliance of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameterMeters * Math.PI; //Calculates the conversion factor of RPM of the translation motor to m/s at the floor

    //NOTE: You shoulds ALWAYS define a reasonable current limit when using brushless motors 
    //      due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; //Defines the PID values for rotation of the serve modules, should show some minor oscillation when no weight is loaded on the modules
  }
  
  /**
   * Vision/Limelight constants
   */
  public static final class VisionConstants {
    public static final double kElevationOffsetDegrees = 38.5;            // Degree offset of lens from horizontal due to camera mount
    public static final double kAzimuthalAngleDegrees = -0.50;            // Degree azimuthal offset of limelight
    public static final double kTargetCenterHeightFromLensInches = 81.0;  // Center Height of the Target in inches above the lens
    public static final double kTrackToleranceRadians = 0.0200;           // Allowable Limelight angle error in radians
  
    public static final int kVisionLedOn = 0;
    public static final int kVisionLedOff = 1;

    public static final double kLineupSpeed = 0.3;
    public static final double kLineupAccuracy = 2.0;
  }

  /**
   * CANdle constants
   */
  public static final class CandleConstants {
    public static final int kCANdleID    = 1;
    public static final int kMaxBrightnessAngle  = 90;
    public static final int kMidBrightnessAngle  = 180;
    public static final int kZeroBrightnessAngle = 270;
  }

  /**
   * Intake constants 
   */
    public static final class IntakeConstants  {

    public static final int kIntakeEncoderCPR = 42;

    public static final int kRollerGearRatio = 3; // 54 / 18
    public static final double kRollerIntakeSpeed = 1.0;
    
    public static final double kCamGearRatio = 47915 / 486; // 12/74, 18/74, 18/70
    public static final double kCamOpenPose = 2265.0;
    public static final double kCamClosedPose = 0.0;
    public static final double kIntakeSpeed = 1.0;
    
    public static final int kIntakeMode = 1; //0 = Roller, 1 = Clamp

    public static final int kIntakeMotorFrontID = 22;
    public static final int kIntakeMotorBackID  = 23;
    public static final int kIntakeMotorRightID = 25;
    public static final int kIntakeMotorLeftID  = 27;

  }

  
  /**
   * feeder constants 
   */
  public static final class FeederConstants  {
    public static final int kFeederStage1MotorID  = 31;
    public static final int kFeederStage2MotorID  = 32;
  }

  /**
   * Arm constants 
   */
  public static final class ArmConstants  {
    public static final int kArmMaster1MotorID  = 30;
    public static final int kArmSlaveMotorID    = 42;
    public static final double kSetPoint1 = 20.5;
    public static final double kSetPoint2 = 60.5;
    public static final double kThresholdArm = 0.5;
    
  }

 /**
   * Shooter constants 
   */
  public static final class ShooterConstants  {    
    // public static final int kShooterMotorID = 30; //keep as 30 once move arm testing is done
    public static final int kShooterMotorTopID = 51;
    public static final int kShooterMotorBottomID = 52;
    public static final int kShooterTargetSpeed = 1000;

    //speed constants
    public static final double kShooterHighCloseMPS = 1500;
    public static final double kP = 0.00015;
    public static final double kI = 0.000001; 
    public static final double kD = 0.006;
    public static final double kFF = 0.0;
    public static final double kIZone = 0;
    public static final double kMin = -1;
    public static final double kMax = 1;
    public static final double kThreshhold = 0.75;
  }


  /**
    * User Controller constants 
    */
  public static final class ControllerConstants {
    public static final int kDriverControllerPort     = 0;
    public static final int kOperatorControllerPort   = 1;
    
    // Driver
    public static final double kDriverDeadBandLeftX   = 0.1;
    public static final double kDriverDeadBandRightX  = 0.2;
    public static final double kDriverDeadBandLeftY   = 0.1;
    public static final double kDriverDeadBandRightY  = 0.2;

    // Operator
    // public static final double kOperatorDeadBandLeftX   = 0.1;
    // public static final double kOperatorDeadBandRightX  = 0.2;
    // public static final double kOperatorDeadBandLeftY   = 0.1;
    // public static final double kOperatorDeadBandRightY  = 0.2;
  }

  /**
    * Auto constants 
    */
  public static final class AutoConstants {
    public static final double kMaxAcceleration = 6.00;
    public static final double kMaxSpeed = 8.0; //Maximum Sustainable Drivetrain Speed under Normal Conditions & Battery, Robot will not exceed this speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    public static final double kMaxAngularAccel = Math.PI;      //Maximum Angular Speed desired. NOTE: Robot can exceed this but spinning fast is not particularly useful or driver friendly
    
    public static final double kPXController = 3.0;
    public static final double kPYController = 3.0;
    public static final double kPThetaController = 3.0;
    
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeed, kMaxAngularAccel); //Creates a trapezoidal motion for the auto rotational commands
  }

  public static final class VisionProcessorConstants {

    public static final int CANdleID = 1;

    public static final int kVisionLedOn = 0;
    public static final int kVisionLedOff = 1;

    // (50*38)/(12*20) - Lowspeed
    // (44*38)/(12*26) - Highspeed

  }
  
  /**
  * Simulation constants 
  */
  public static final class SimConstants {
    //Game Specific
  }
  
  /**
  * FMS constants 
  */
  public static final class FMSConstants {
    public static final int ALLIANCE_RED 	 		    = 1;
    public static final int ALLIANCE_BLUE 	 		  = -1;
    public static final int ALLIANCE_INITIALIZED  = 0;
    public static final int ALLIANCE_NOT_ENABLED  = 20;
    public static final int ALLIANCE_EXCEPTION    = 11;
  }
}
