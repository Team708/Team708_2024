// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ArmConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotorTop;//master shooter motor
  private CANSparkMax m_shooterMotorBottomFollower;//slave shooter motor
  public RelativeEncoder shooterEncoderTop;//sets encoder for master motor

  private SparkPIDController shooterTopPIDController;//PID for master shooter
  public double targetSpeed = 0;

  private CANSparkMax m_PivotArmTop;//master pivot arm motor
  private CANSparkMax m_PivotArmTopFollower; //slave pivot arm motor
  private RelativeEncoder PivotArmTopEncoder;

  private SparkPIDController PivotArmTopPIDController;
  public double pivotTargetSpeed = 0;

  private double currentArmPosition;
  private double setPoint;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorTop = new CANSparkMax(ShooterConstants.kShooterMotorTopID, MotorType.kBrushless);
    m_shooterMotorTop.setIdleMode(IdleMode.kCoast);
    m_shooterMotorTop.setInverted(false);

    shooterEncoderTop = m_shooterMotorTop.getEncoder();
    
    shooterTopPIDController = m_shooterMotorTop.getPIDController();
    shooterTopPIDController.setP(ShooterConstants.kP);
    shooterTopPIDController.setI(ShooterConstants.kI);
    shooterTopPIDController.setD(ShooterConstants.kD);
    shooterTopPIDController.setFF(ShooterConstants.kFF);
    shooterTopPIDController.setIZone(ShooterConstants.kIZone);
    shooterTopPIDController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax);

    
    m_shooterMotorBottomFollower = new CANSparkMax(ShooterConstants.kShooterMotorBottomID, MotorType.kBrushless);
    m_shooterMotorBottomFollower.setIdleMode(IdleMode.kCoast);
    m_shooterMotorBottomFollower.follow(m_shooterMotorTop, true);

    //now create arm master stuff
    m_PivotArmTop = new CANSparkMax(ArmConstants.kArmMaster1MotorID, MotorType.kBrushless);
    m_PivotArmTop.setIdleMode(IdleMode.kCoast);
    m_PivotArmTop.setInverted(false);

    PivotArmTopEncoder = m_PivotArmTop.getEncoder();

    PivotArmTopPIDController = m_PivotArmTop.getPIDController();
    PivotArmTopPIDController.setP(ShooterConstants.kP);
    shooterTopPIDController.setI(ShooterConstants.kI);
    shooterTopPIDController.setD(ShooterConstants.kD);
    shooterTopPIDController.setFF(ShooterConstants.kFF);
    shooterTopPIDController.setIZone(ShooterConstants.kIZone);
    shooterTopPIDController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax);

    //now create arm slave stuff
    
    m_PivotArmTopFollower = new CANSparkMax(ArmConstants.kArmSlaveMotorID, MotorType.kBrushless);
    m_PivotArmTopFollower.setIdleMode(IdleMode.kCoast);
    m_PivotArmTopFollower.follow(m_PivotArmTop, false);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooterVelocitySpeaker(double velocity){
    targetSpeed = velocity;
    shooterTopPIDController.setReference(velocity, CANSparkBase.ControlType.kVelocity);
  }

  public void shooterVelocityAmp(double velocity){
    targetSpeed = velocity;
    m_shooterMotorBottomFollower.follow(m_shooterMotorTop, false);
    shooterTopPIDController.setReference(velocity, CANSparkBase.ControlType.kVelocity);
  }

  public void stopShooter(){
    targetSpeed = 0;
    shooterTopPIDController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public boolean isShooterAtSpeed() {
    if ((Math.abs(shooterEncoderTop.getVelocity()) > (targetSpeed) * ShooterConstants.kThreshhold)){
      return true;
    }else{
      return false;
    }
  }

  //Now we want to get arm from current position to target setpoint

  public double findDisplacement(double setPoint) {
    double currentArmPosition = PivotArmTopEncoder.getPosition();
    return (setPoint - currentArmPosition);
  }

  public void moveArm(double setPoint) {
    PivotArmTopPIDController.setReference(setPoint, CANSparkBase.ControlType.kPosition);
  }

  public void sendToDashboardSetPoint1() {
    SmartDashboard.putNumber("displacement values", findDisplacement(ArmConstants.kSetPoint1));
  }

}
