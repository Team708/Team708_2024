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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utilities.Helper;

public class PivotArm extends SubsystemBase {
  private CANSparkMax m_PivotArmLeftLeader, m_PivotArmRightFollower;
  private RelativeEncoder PivotArmEncoder;
  private SparkPIDController PivotArmPIDController;
  private double targetArmAngle;
  
  public PivotArm() {
    //Leader arm motor
    m_PivotArmLeftLeader = new CANSparkMax(ArmConstants.kArmMasterMotorID, MotorType.kBrushless);
    m_PivotArmLeftLeader.setIdleMode(IdleMode.kCoast);
    m_PivotArmLeftLeader.setInverted(false);

    PivotArmEncoder = m_PivotArmLeftLeader.getEncoder();
    
    PivotArmPIDController = m_PivotArmLeftLeader.getPIDController();
    Helper.setupPIDController(PivotArmPIDController, ArmConstants.kPivotArmPIDList);

    //Follower arm motor
    m_PivotArmRightFollower = new CANSparkMax(ArmConstants.kArmSlaveMotorID, MotorType.kBrushless);
    m_PivotArmRightFollower.setIdleMode(IdleMode.kCoast);
    m_PivotArmRightFollower.follow(m_PivotArmLeftLeader, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //make sure units are correct
  public double findDisplacement(double setPoint) {
    double currentArmPosition = PivotArmEncoder.getPosition();
    return (setPoint - currentArmPosition);
  }

  //Determine units for arm as it's not completely tested
  public void setArmAngle(double angle) {
    PivotArmPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);
  }

  public double findArmAngle (double distanceFromSpeaker) {
    //This is where we add the equations to solve for targetArmAngle
    
    targetArmAngle = ArmConstants.kBumperShotAngle; //Using constant temporarily
    return targetArmAngle;
  }
}
