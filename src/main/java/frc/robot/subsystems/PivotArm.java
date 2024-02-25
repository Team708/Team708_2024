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
  private SparkLimitSwitch forwardLimit, reverseLimit;
  private Drivetrain m_drive;
  private Boolean isTargeting = false;
  private double distance;
  private InterpolatingDoubleTreeMap interpolatingTreeMap = new InterpolatingDoubleTreeMap();

  public PivotArm(Drivetrain drive) {
    m_drive = drive;

    absEncoder = new DutyCycleEncoder(0);
    //absEncoder.reset();
    
    //Leader arm motor
    m_PivotArmLeftLeader = new CANSparkMax(ArmConstants.kArmMasterMotorID, MotorType.kBrushless);
    m_PivotArmLeftLeader.setIdleMode(IdleMode.kBrake);
    m_PivotArmLeftLeader.setSmartCurrentLimit(CurrentLimit.kArmAmps);
    m_PivotArmLeftLeader.setInverted(false);
    
    forwardLimit = m_PivotArmLeftLeader.getForwardLimitSwitch(Type.kNormallyOpen);
    forwardLimit.enableLimitSwitch(true);
    reverseLimit = m_PivotArmLeftLeader.getReverseLimitSwitch(Type.kNormallyOpen);
    reverseLimit.enableLimitSwitch(true);
    
    PivotArmEncoder = m_PivotArmLeftLeader.getEncoder();
    PivotArmEncoder.setPositionConversionFactor(ArmConstants.kPivotArmGearRatio);
    PivotArmEncoder.setPosition(getAbsolutePosition());
    
    pivotArmPIDController = m_PivotArmLeftLeader.getPIDController();
    PidHelper.setupPIDController(this.getName()+"pivotArmPIDController", pivotArmPIDController, ArmConstants.kPivotArmPIDList);
    
    //Follower arm motor
    m_PivotArmRightFollower = new CANSparkMax(ArmConstants.kArmSlaveMotorID, MotorType.kBrushless);
    m_PivotArmRightFollower.setIdleMode(IdleMode.kBrake);
    m_PivotArmRightFollower.setSmartCurrentLimit(40);
    m_PivotArmRightFollower.follow(m_PivotArmLeftLeader, true);

    //data potins f
    interpolatingTreeMap.put(1.3, 48.0);
    interpolatingTreeMap.put(1.94, 38.5);
    interpolatingTreeMap.put(2.5, 32.4);
    interpolatingTreeMap.put(2.8, 32.2);
    interpolatingTreeMap.put(3.3, 30.07);
    interpolatingTreeMap.put(3.7,25.9);
    interpolatingTreeMap.put(4.0,24.7);
    interpolatingTreeMap.put(4.3,24.5);
    interpolatingTreeMap.put(4.8,23.9);
    interpolatingTreeMap.put(5.6,23.45);
    interpolatingTreeMap.put(6.2,23.22);

  }
  
  @Override
  public void periodic() {
    
  }
  
  //current and voltage limits
  
  public double getPosition() {
    return PivotArmEncoder.getPosition();
  }
  
  public double getAbsolutePosition() {
    return (-ArmConstants.kArmClockingOffset-(absEncoder.getAbsolutePosition()*ArmConstants.kArmScalingFactor)-ArmConstants.kArmAbsEncoderOffset);
  }

  public double inputTransform(double input) {
        //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
        return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
    }

  public void operateByController() {
    double desiredY = -inputTransform(OI.getOperatorLeftY());
    if(Math.abs(desiredY) > ControllerConstants.kOperatorDeadBandLeftY) {
      setArmAngle(getPosition()+(desiredY*2));
    }
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
    distance = m_drive.getDistanceToTarget();
    if(distance < ArmConstants.kMaxShootingDistance){
      return interpolatingTreeMap.get(distance);
    }
    return 0;
  }

  public boolean isArmAtPosition() {
      return Math.abs(targetArmAngle - getPosition()) < ArmConstants.kThresholdArm;
    }
  

  public void sendToDashboard() {
    String topic = new String(this.getName()+"/");
    SmartDashboard.putNumber(topic+"Arm Position", getPosition());
    SmartDashboard.putNumber(topic+"Arm Abs Position", getAbsolutePosition());
	  SmartDashboard.putBoolean(topic+"Arm At Position", isArmAtPosition());
    SmartDashboard.putBoolean(topic+"Arm Forward Limit", forwardLimit.isPressed());
    SmartDashboard.putBoolean(topic+"Arm Reverse Limit", reverseLimit.isPressed());
  }
}
