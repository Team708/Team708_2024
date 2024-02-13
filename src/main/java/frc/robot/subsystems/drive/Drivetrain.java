// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.OI;
import frc.robot.commands.DriveByController;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;
import frc.robot.utilities.MathUtils;

  /**
   * Implements a swerve Drivetrain Subsystem for the Robot
   */
  public class Drivetrain extends SubsystemBase {

  //Create the PIDController for the Keep Angle PID
  private final PIDController m_keepAnglePID = new PIDController(DriveConstants.kKeepAnglePID[0],
    DriveConstants.kKeepAnglePID[1],DriveConstants.kKeepAnglePID[2]);
  
  private double keepAngle = 0.0;       //Double to store the current target keepAngle in radians
  private double timeSinceRot = 0.0;    //Double to store the time since last rotation command
  private double lastRotTime = 0.0;     //Double to store the time of the last rotation command
  private double timeSinceDrive = 0.0;  //Double to store the time since last translation command
  private double lastDriveTime = 0.0;   //Double to store the time of the last translation command
  
  private double radius = 1;//0.450;

  private boolean m_readyToShoot = false;

  private boolean fieldOrient = true;

  private final SlewRateLimiter m_slewX = new SlewRateLimiter(12.0);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(12.0);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(20.0);

  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

  private final Timer keepAngleTimer = new Timer(); //Creates timer used in the perform keep angle function

  private boolean autoRotEnabled = false;

  private PathConstraints trajectoryConstraints;

  //Creates a swerveModule object for the front left swerve module feeding in parameters from the constants file
  private final SwerveModule m_frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftOffset, DriveConstants.kFrontLeftTuningVals);

  //Creates a swerveModule object for the front right swerve module feeding in parameters from the constants file
  private final SwerveModule m_frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightOffset, DriveConstants.kFrontRightTuningVals);

  //Creates a swerveModule object for the back left swerve module feeding in parameters from the constants file
  private final SwerveModule m_backLeft = new SwerveModule(DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftTurningEncoderPort,
      DriveConstants.kBackLeftOffset, DriveConstants.kBackLeftTuningVals);

  //Creates a swerveModule object for the back right swerve module feeding in parameters from the constants file
  private final SwerveModule m_backRight = new SwerveModule(DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightTurningEncoderPort,
      DriveConstants.kBackRightOffset, DriveConstants.kBackRightTuningVals);
  
  //Get pigeon gyro instance
  private static PigeonTwo pigeon = PigeonTwo.getInstance();

  //Creates Odometry object to store the pose of the robot
  public final SwerveDrivePoseEstimator m_PoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, pigeon.getAngle(), getModulePositions(), DriveConstants.kinitialPoseMeters);

  private final SwerveDrivePoseEstimator m_AutoPoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, pigeon.getAngle(), getModulePositions(), DriveConstants.kinitialPoseMeters);

  ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
    AutoConstants.kThetaControllerConstraints);

  private final Field2d m_field;

  private Pose2d currentPose = new Pose2d();
  private Pose2d targetPose = new Pose2d();

  /**
   * Constructs a Drivetrain and resets the Gyro and Keep Angle parameters
   */
  public Drivetrain() {
    keepAngleTimer.reset();
    keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    pigeon.reset();
    // m_PoseEstimator.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), new Pose2d()); //JNP 
    m_PoseEstimator.resetPosition(pigeon.getAngle().times(1.0), getModulePositions(), new Pose2d()); //JNP 
    CommandScheduler.getInstance().registerSubsystem(this);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_field = new Field2d();

    SmartDashboard.putData("Field", m_field);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean keepAngle) {
    // SmartDashboard.putBoolean("fieldRelative", fieldRelative);
    // SmartDashboard.putBoolean("keepAngle", keepAngle);

    // if(keepAngle){
    //   rot = performKeepAngle(xSpeed,ySpeed,rot); //Calls the keep angle function to update the keep angle or rotate depending on driver input
    // }
    
    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);
    rot = m_slewRot.calculate(rot);
    
    // SmartDashboard.putNumber("xSpeed Commanded", xSpeed);
    // SmartDashboard.putNumber("ySpeed Commanded", ySpeed);
    // SmartDashboard.putNumber("rot Commanded", rot);

    //creates an array of the desired swerve module states based on driver command and if the commands are field relative or not
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getAngle())
            : new ChassisSpeeds(xSpeed * pigeon.getAngle().getCos() + ySpeed * pigeon.getAngle().getSin(), 
            (-xSpeed * pigeon.getAngle().getSin() + ySpeed * pigeon.getAngle().getCos()) - (radius * rot), 
            rot));

    //normalize wheel speeds so all individual states are scaled to achievable velocities
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSec); 

    setModuleStates(swerveModuleStates);
  }

  @Override
  public void periodic(){
      m_fieldRelVel = new FieldRelativeSpeed(getChassisSpeed(), getGyro());
      m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, GlobalConstants.kLoopTime);
      //m_fieldRelJerk = new FieldRelativeJerk(m_fieldRelAccel, m_lastFieldRelAccel, GlobalConstants.kLoopTime);
      //m_lastFieldRelAccel = m_fieldRelAccel;
      m_lastFieldRelVel = m_fieldRelVel;

      // SmartDashboard.putNumber("RobotSpeedX", getChassisSpeed().vxMetersPerSecond);
      // SmartDashboard.putNumber("RobotSpeedY", getChassisSpeed().vyMetersPerSecond);
      // SmartDashboard.putNumber("RobotOmega", getChassisSpeed().omegaRadiansPerSecond);

      // SmartDashboard.putNumber("Robot pitch degrees", pigeon.getPitch().getDegrees());
      // SmartDashboard.putNumber("Robot roll degrees", pigeon.getRoll().getDegrees());

      // SmartDashboard.putNumber("Accel X", m_fieldRelAccel.ax);
      // SmartDashboard.putNumber("Accel Y", m_fieldRelAccel.ay);
      // SmartDashboard.putNumber("Alpha", m_fieldRelAccel.alpha);

        SmartDashboard.putNumber("Front Left Encoder", m_frontLeft.getTurnEncoder());
        SmartDashboard.putNumber("Front Right Encoder", m_frontRight.getTurnEncoder());
        SmartDashboard.putNumber("Back Left Encoder", m_backLeft.getTurnEncoder());
        SmartDashboard.putNumber("Back Right Encoder", m_backRight.getTurnEncoder());

        SmartDashboard.putNumber("Balance Angle", pigeon.getRoll().getDegrees());

        //Update swerve drive odometry periodically so robot pose can be tracked
        updatePose();    

        //Calls get pose function which sends the Pose information to the SmartDashboard
        currentPose = getPose();
 
        m_field.getRobotObject().setPose(currentPose);
  }

  public void setFieldOrient(boolean fieldOrient){
    this.fieldOrient = fieldOrient;
  }

  public boolean getFieldOrient(){
    return fieldOrient;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSec);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    };
  }

  public void stop(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  /**
   * Updates odometry for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */  
  public void updatePose() {
    m_PoseEstimator.update(pigeon.getAngle(), getModulePositions());
  }

  public void updateAutoPose() {
    m_AutoPoseEstimator.update(pigeon.getAngle(), getModulePositions());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * @return Rotation2d object containing Gyro angle
   */  
  public Rotation2d getGyro() {
    return pigeon.getAngle();
  }

  public double getGyroDegrees() {
    return pigeon.getAngle().getDegrees();
  }

  public FieldRelativeSpeed getFieldRelativeSpeed(){
    return m_fieldRelVel;
  }

  public FieldRelativeAccel getFieldRelativeAccel(){
    return m_fieldRelAccel;
  }

    /**
   * Function created to retreieve and push the robot pose to the SmartDashboard for diagnostics
   * @return Pose2d object containing the X and Y position and the heading of the robot.
   */  
  public Pose2d getPose() {
    Pose2d pose = m_PoseEstimator.getEstimatedPosition();
    Translation2d position = pose.getTranslation();
    //Rotation2d heading = getGyro();
    SmartDashboard.putNumber("Robot X", position.getX());
    SmartDashboard.putNumber("Robot Y", position.getY());
    //SmartDashboard.putNumber("Robot Gyro", getGyro().getDegrees());
    SmartDashboard.putNumber("Robot Angle", pose.getRotation().getDegrees());
    return pose;
  }

  public Pose2d getAutoPose() {
    updateAutoPose();
    Pose2d pose = m_AutoPoseEstimator.getEstimatedPosition();
    Translation2d position = pose.getTranslation();
    SmartDashboard.putNumber("Auto X", position.getX());
    SmartDashboard.putNumber("Auto Y", position.getY());
    return pose;
  }

  /**
   * Resets the odometry and gyro to the specified pose.
   *
   * @param pose in which to set the odometry and gyro.
   */
  public void resetOdometry(Pose2d pose) {
    // pigeon.reset();
    // pigeon.setAngle(pose.getRotation().getDegrees());
    keepAngle = getGyro().getRadians();

    m_PoseEstimator.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);
    m_AutoPoseEstimator.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);
  }

  public void setPose(Pose2d pose){
    m_PoseEstimator.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);
        keepAngle = getGyro().getRadians();
  }


    /**
   * Resets the gyro to the given angle
   * 
   * @param angle the angle of the robot to reset to
   */
  public void resetOdometry(Rotation2d angle) {
    Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
    pigeon.reset();
    pigeon.setAngle(angle.getDegrees());
    keepAngle = getGyro().getRadians();
    m_PoseEstimator.resetPosition(pigeon.getAngle().times(-1.0), getModulePositions(), pose);  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the swerve drive kinematics.
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity 
   */  
  public ChassisSpeeds getChassisSpeed(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
    m_backRight.getState());
  }

  /**
   * Keep angle function is performed to combat drivetrain drift without the need of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the keepAngle value. This value is updated when the robot 
   * is rotated manually by the driver input
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot is the input drive rotation speed command
   */  
  private double performKeepAngle(double xSpeed, double ySpeed, double rot){
    double output = rot; //Output should be set to the input rot command unless the Keep Angle PID is called
    if(Math.abs(rot) >= DriveConstants.kMinRotationCommandRadPerSec){  //If the driver commands the robot to rotate set the last rotate time to the current time
      lastRotTime = keepAngleTimer.get();
    }
    if( Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommandMetersPerSec  
          || Math.abs(ySpeed) >= DriveConstants.kMinTranslationCommandMetersPerSec){ //if driver commands robot to translate set the last drive time to the current time
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get()-lastRotTime;      //update variable to the current time - the last rotate time
    timeSinceDrive = keepAngleTimer.get()-lastDriveTime;  //update variable to the current time - the last drive time
    if(timeSinceRot < 0.5){                               //Update keepAngle up until 0.5s after rotate command stops to allow rotation move to finish
      keepAngle = getGyro().getRadians();
    }   //JNP anythingn here to tweek rotate speed??
    else if(Math.abs(rot) < DriveConstants.kMinRotationCommandRadPerSec && timeSinceDrive < 0.25){ //Run Keep angle pid until 0.75s after drive command stops to combat decel drift
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle);               //Set output command to the result of the Keep Angle PID 
    }
    return output;
  }

  public void updateKeepAngle(){
    keepAngle = getGyro().getRadians();
  }

  public void setReadytoShoot(){
    m_readyToShoot = true;
  }

  public boolean isReadyToShoot(){
    return m_readyToShoot;
  }

  private SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(),
      m_backRight.getPosition()};
  }

  public void invertDrive(){
    m_frontLeft.invertDrive();
    m_frontRight.invertDrive();
    m_backLeft.invertDrive();
    m_backRight.invertDrive();
  }

  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
      path,
      this::getPose, // Robot pose supplier
      this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        4.5, // Max module speed, in m/s
        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds)
  {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSec);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
  
  public Translation2d getDriverXAndY()
  {
    double maxLinear = DriveConstants.kMaxSpeedMetersPerSec;
    double desiredX = -inputTransform(OI.getDriverLeftY())*maxLinear;
    double desiredY = -inputTransform(OI.getDriverLeftX())*maxLinear;
    Translation2d desiredTranslation = new Translation2d(desiredX, desiredY);
    double desiredMag = desiredTranslation.getDistance(new Translation2d());
    if(desiredMag >= maxLinear){
      desiredTranslation.times(maxLinear/desiredMag);
    }
    return desiredTranslation;
  }

  public double getDriverRot()
  {
    return -inputTransform(OI.getDriverRightX())* DriveConstants.kMaxAngularSpeedRadPerSec;
  }

  private double inputTransform(double input){
    //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
  }

  public void makeRobotDrive()
  {
    PIDController controller = Constants.DriveConstants.kAutoRotatePID;//, //new Constraints(300000, 150000));
    double desiredRot = findAutoRotate(controller, getDriverRot());
    this.drive(getDriverXAndY().getX(), 
                       getDriverXAndY().getY(),
                       desiredRot,
                       getFieldOrient(),
                       true);
  }

  public void enableAutoRot()
  {
    autoRotEnabled = true;
  }

  public void disableAutoRot()
  {
    autoRotEnabled = false;
  }

  public double findAutoRotate(PIDController controller, double defaultRot)
  {
      if(autoRotEnabled){
        targetPose = DriveConstants.kBlueSpeaker;
        double dx = targetPose.getX() - getPose().getX();
        double dy = targetPose.getY() - getPose().getY();
        Rotation2d robotToTarget = new Rotation2d(dx, dy);
        m_field.getObject("Desired Target").setPose(targetPose);
        double output =  controller.calculate(getPose().getRotation().getDegrees(), robotToTarget.getDegrees());
        if(!controller.atSetpoint()){
          return output;
        }else{
          return 0.0;
        }
      }
      return defaultRot;  
  }

  public double getDistanceToTarget() {
    return getPose().getTranslation().getDistance(DriveConstants.kBlueSpeaker.getTranslation());
  }

  public Trajectory createTrajectory(Pose2d desiredPose)
  {
    TrajectoryConfig config = new TrajectoryConfig(Constants.DriveConstants.kMaxSpeedMetersPerSec, Constants.DriveConstants.kMaxAccelMetersPerSecSquared);
    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    waypoints.add(getPose());
    waypoints.add(desiredPose);
    return TrajectoryGenerator.generateTrajectory(waypoints, config);
  }

  public void driveToPoint(Pose2d desiredLocation, Pose2d trueEndingLocation) {
    trajectoryConstraints = new PathConstraints(
    DriveConstants.kMaxSpeedMetersPerSec,
    DriveConstants.kMaxAccelMetersPerSecSquared, 
    DriveConstants.kMaxAngularSpeedRadPerSec, 
    DriveConstants.kMaxAngularAccel);
    
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(getPose(), desiredLocation.transformBy(new Transform2d(0,0,new Rotation2d(Math.PI))));
    PathPlannerPath path = new PathPlannerPath(
      bezierPoints, 
      trajectoryConstraints,  
      new GoalEndState(0.0, desiredLocation.getRotation())
    );

    List<Translation2d> bezierPoints2 = PathPlannerPath.bezierFromPoses(desiredLocation.transformBy(new Transform2d(0,0,new Rotation2d(Math.PI))), trueEndingLocation.transformBy(new Transform2d(0,0,new Rotation2d(Math.PI))));
    PathPlannerPath path2 = new PathPlannerPath(
      bezierPoints2, 
      trajectoryConstraints,  
      new GoalEndState(0.0, trueEndingLocation.getRotation())
    );

    // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();
    AutoBuilder.followPath(path2).schedule();
    System.out.println("ran init");
  }

  public void sendToDashboard() {
  }
  
}