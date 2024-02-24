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

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
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

  public PivotArm(Drivetrain drive) {
    m_drive = drive;

    absEncoder = new DutyCycleEncoder(0);
    absEncoder.reset();
    
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
    m_PivotArmLeftLeader.set(desiredY);

  }
  //Determine units for arm as it's not completely tested
  public void setArmAngle(double angle) {
    targetArmAngle = angle;
    pivotArmPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);    
  }
  
  public double findArmAngle () {
    double distance = m_drive.getDistanceToTarget();
    if(distance < ArmConstants.kMaxShootingDistance) {
      //This is where we add the equations to solve for targetArmAngle
      targetArmAngle = 1.86876*Math.pow(distance, 2) -19.9052*distance + 73.639; //Using constant temporarily
      return targetArmAngle;
    }
    else {
      return 0;
    }
  }

  public boolean isArmAtPosition() {
    return (targetArmAngle- getPosition()) < ArmConstants.kThresholdArm;
  }

  public void sendToDashboard() {
    String topic = new String("/"+this.getName()+"/");
    SmartDashboard.putNumber(topic+"Arm Encoder Position", getPosition());
    SmartDashboard.putNumber(topic+"Absolute Encoder Position", getAbsolutePosition());
    SmartDashboard.putBoolean(topic+"Arm Forward Limit", forwardLimit.isPressed());
    SmartDashboard.putBoolean(topic+"Arm Reverse Limit", reverseLimit.isPressed());
  }
}
