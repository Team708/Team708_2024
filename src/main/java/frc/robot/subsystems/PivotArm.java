// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utilities.PidHelper;
import frc.robot.utilities.MathUtils;
import frc.robot.OI;
public class PivotArm extends SubsystemBase {
  private CANSparkMax m_PivotArmLeftLeader, m_PivotArmRightFollower;
  private RelativeEncoder PivotArmEncoder;
  private DutyCycleEncoder absEncoder;
  private SparkPIDController pivotArmPIDController;
  private double targetArmAngle;
  // private SparkLimitSwitch forwardLimit, reverseLimit;
  private Drivetrain m_drive;
  private Boolean isTargeting = false;
  private double distance;
  private InterpolatingDoubleTreeMap interpolatingTreeMap = new InterpolatingDoubleTreeMap();
  private boolean lastIdleModeState = false;

  public PivotArm(Drivetrain drive) {
    m_drive = drive;

    absEncoder = new DutyCycleEncoder(0);
    //absEncoder.reset();
    
    //Leader arm motor
    m_PivotArmLeftLeader = new CANSparkMax(ArmConstants.kArmMasterMotorID, MotorType.kBrushless);
    m_PivotArmLeftLeader.setIdleMode(IdleMode.kBrake);
    m_PivotArmLeftLeader.setSmartCurrentLimit(CurrentLimit.kArmAmps);
    m_PivotArmLeftLeader.setInverted(false);
    
    // forwardLimit = m_PivotArmLeftLeader.getForwardLimitSwitch(Type.kNormallyOpen);
    // forwardLimit.enableLimitSwitch(true);
    // reverseLimit = m_PivotArmLeftLeader.getReverseLimitSwitch(Type.kNormallyOpen);
    // reverseLimit.enableLimitSwitch(true);
    
    PivotArmEncoder = m_PivotArmLeftLeader.getEncoder();
    PivotArmEncoder.setPositionConversionFactor(ArmConstants.kPivotArmGearRatio);
    PivotArmEncoder.setPosition(getAbsolutePosition());
    
    pivotArmPIDController = m_PivotArmLeftLeader.getPIDController();
    PidHelper.setupPIDController(this.getName()+"pivotArmPIDController", pivotArmPIDController, ArmConstants.kPivotArmPIDList);
    
    //Follower arm motor
    m_PivotArmRightFollower = new CANSparkMax(ArmConstants.kArmSlaveMotorID, MotorType.kBrushless);
    m_PivotArmRightFollower.setIdleMode(IdleMode.kBrake);
    m_PivotArmRightFollower.setSmartCurrentLimit(CurrentLimit.kArmAmps);
    m_PivotArmRightFollower.follow(m_PivotArmLeftLeader, true);

    //data potints outdated 
//     interpolatingTreeMap.put(1.27,47.8);//smp
//     interpolatingTreeMap.put(1.3, 48.0);
//     interpolatingTreeMap.put(1.4,45.5);//smp
//     interpolatingTreeMap.put(1.94, 38.5);
//     interpolatingTreeMap.put(2.2,32.8);//smp
// //  interpolatingTreeMap.put(2.5, 32.4);
//     interpolatingTreeMap.put(2.7,27.9);//smp
// //  interpolatingTreeMap.put(2.8, 32.2);
// //  interpolatingTreeMap.put(3.3, 30.07);
//     interpolatingTreeMap.put(3.3,26.9);//smp
// //  interpolatingTreeMap.put(3.5, 28.0);  //Added

//     interpolatingTreeMap.put(3.7,25.9);
//     interpolatingTreeMap.put(4.0,24.7);
//     interpolatingTreeMap.put(4.3,24.5);
//     interpolatingTreeMap.put(4.6,21.6);//smp
// //  interpolatingTreeMap.put(4.8,23.9);
// //  interpolatingTreeMap.put(5.6,23.45);
//     interpolatingTreeMap.put(5.7,18.5);//smp
//    interpolatingTreeMap.put(6.2,23.22);



//Data points maded 02.27 precomp
interpolatingTreeMap.put(1.22,53.0);
interpolatingTreeMap.put(1.945,44.0); //42.5
interpolatingTreeMap.put(2.244,41.0); //39.8
interpolatingTreeMap.put(2.87,35.0); //33.5
interpolatingTreeMap.put(3.075,32.0);  //3.07, 33.0 //29.1
interpolatingTreeMap.put(3.61,30.0);
interpolatingTreeMap.put(3.8,28.5); //3.77,31.08 //25.4
interpolatingTreeMap.put(4.073,25.73); //4.06, 29.17
interpolatingTreeMap.put(4.91,24.157); //4.84, 26.70
interpolatingTreeMap.put(5.92,23.78);  //5.61, 24.70

}
  
  @Override
  public void periodic() {
    
  }
  
  //current and voltage limits
  
  public double getPosition() {
    // return (ArmConstants.kArmClockingOffset-(absEncoder.getAbsolutePosition()*ArmConstants.kArmScalingFactor)-ArmConstants.kArmAbsEncoderOffset);
    //Replaced to avoid searching for every instance of the method. DO NOT CHANGE WITHOUT APPROVAL FROM JOHN
    return PivotArmEncoder.getPosition();
  }
  
  public double getAbsolutePosition() {
    return (-ArmConstants.kArmClockingOffset-(absEncoder.getAbsolutePosition()*ArmConstants.kArmScalingFactor)-ArmConstants.kArmAbsEncoderOffset);   //Previous 
    // return (ArmConstants.kArmClockingOffset-(absEncoder.getAbsolutePosition()*ArmConstants.kArmScalingFactor)-ArmConstants.kArmAbsEncoderOffset);
  }

  public double inputTransform(double input) {
        //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
        return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
    }

  public void operateByController() {
    double desiredY = -inputTransform(OI.getOperatorLeftY());
    if(Math.abs(desiredY) > ControllerConstants.kOperatorDeadBandLeftY) {
      setArmAngle(getPosition()+(desiredY*3));
    }
    // if(reverseLimit.isPressed()) {
    //   PivotArmEncoder.setPosition(getAbsolutePosition());
    // }
  }
  
  //Determine units for arm as it's not completely tested
  public void setArmAngle(double angle) {
    targetArmAngle = angle;
    pivotArmPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);    
  }
  
  // public double findArmAngle () {
  //   double distance = m_drive.getDistanceToTarget();
  //   if(distance < ArmConstants.kMaxShootingDistance) {
  //     //This is where we add the equations to solve for targetArmAngle
  //     isTargeting = true;
  //     targetArmAngle = 1.86876*Math.pow(distance, 2) -19.9052*distance + 73.639; //Using constant temporarily
  //     return targetArmAngle;
      
  //   }
  //   else {
  //     isTargeting = false;
  //     return 0;
  //   }
  // }
  
  public double findArmAngle(){
    distance = MathUtils.roundDouble(m_drive.getDistanceToTarget(), 2);
    if(distance < ArmConstants.kMaxShootingDistance){
      return interpolatingTreeMap.get(distance);
    }
    return 0;
  }

  public boolean isArmAtPosition() {
      return Math.abs(targetArmAngle - getPosition()) < ArmConstants.kThresholdArm;
    }

  public void setCoastMode(boolean isCoastMode) {
    if (isCoastMode == true && lastIdleModeState != isCoastMode){
      m_PivotArmLeftLeader.setIdleMode(IdleMode.kCoast);
      m_PivotArmRightFollower.setIdleMode(IdleMode.kCoast);
      lastIdleModeState = true;
    }else if (isCoastMode == false && lastIdleModeState != isCoastMode) {
      m_PivotArmLeftLeader.setIdleMode(IdleMode.kBrake);
      m_PivotArmRightFollower.setIdleMode(IdleMode.kBrake);
      lastIdleModeState = false;
    }
  }

  public void sendToDashboard() {
    // String topic = new String(this.getName()+"/");
    SmartDashboard.putNumber("Arm Position", getPosition());
    SmartDashboard.putNumber("Arm Distance", distance);
    // SmartDashboard.putNumber("Arm Abs Position", getAbsolutePosition());   //Arm position is already abs
	  SmartDashboard.putBoolean("Arm At Position", isArmAtPosition());
    
    // SmartDashboard.putNumber(topic+"Arm Amps", m_PivotArmLeftLeader.getOutputCurrent());
    // SmartDashboard.putBoolean(topic+"Arm Forward Limit", forwardLimit.isPressed());
    // SmartDashboard.putBoolean(topic+"Arm Reverse Limit", reverseLimit.isPressed());
  }
}
