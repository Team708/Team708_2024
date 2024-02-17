// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utilities.Helper;

public class PivotArm extends SubsystemBase {
  private CANSparkMax m_PivotArmLeftLeader, m_PivotArmRightFollower;
  private RelativeEncoder PivotArmEncoder;
  private DutyCycleEncoder absEncoder;
  private SparkPIDController PivotArmPIDController;
  private double targetArmAngle;
  
  public PivotArm() {
    absEncoder = new DutyCycleEncoder(0);
    absEncoder.reset();

    //Leader arm motor
    m_PivotArmLeftLeader = new CANSparkMax(ArmConstants.kArmMasterMotorID, MotorType.kBrushless);
    m_PivotArmLeftLeader.setIdleMode(IdleMode.kBrake);
    m_PivotArmLeftLeader.setInverted(true);

    PivotArmEncoder = m_PivotArmLeftLeader.getEncoder();
    PivotArmEncoder.setPositionConversionFactor(ArmConstants.kPivotArmGearRatio);
    PivotArmEncoder.setPosition(getAbsolutePosition());
    
    PivotArmPIDController = m_PivotArmLeftLeader.getPIDController();
    Helper.setupPIDController(PivotArmPIDController, ArmConstants.kPivotArmPIDList);

    //Follower arm motor
    m_PivotArmRightFollower = new CANSparkMax(ArmConstants.kArmSlaveMotorID, MotorType.kBrushless);
    m_PivotArmRightFollower.setIdleMode(IdleMode.kBrake);
    m_PivotArmRightFollower.follow(m_PivotArmLeftLeader, false);

    m_PivotArmLeftLeader.setSmartCurrentLimit(20);
    m_PivotArmRightFollower.setSmartCurrentLimit(20);
  }

  @Override
  public void periodic() {
    
  }

  //current and voltage limits
  
  public double getPosition() {
    return PivotArmEncoder.getPosition();
  }

  public double getAbsolutePosition() {
    return (absEncoder.getAbsolutePosition()*ArmConstants.kArmScalingFactor-ArmConstants.kArmAbsEncoderOffset);
  }
      

  //Determine units for arm as it's not completely tested
  public void setArmAngle(double angle) {
    PivotArmPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);    
  }

  public double findArmAngle () {
    //This is where we add the equations to solve for targetArmAngle
    
    targetArmAngle = ArmConstants.kBumperShotAngle; //Using constant temporarily
    
    return targetArmAngle;
  }

  //make sure units are correct
  public double findDisplacement(double angle) {
    return (angle - getPosition());
  }

  public boolean isArmAtPosition(double angle) {
    return (findDisplacement(angle) < ArmConstants.kThresholdArm);
  }

  public void sendToDashboard() {
    SmartDashboard.putNumber("Arm Encoder Position", getPosition());
    SmartDashboard.putNumber("Absolute Encoder Position", getAbsolutePosition());
  }

}
