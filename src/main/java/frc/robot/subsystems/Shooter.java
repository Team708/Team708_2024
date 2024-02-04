// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
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
  private CANSparkMax m_shooterMotorTopLeader, m_shooterMotorBottomFollower, m_shooterMotorAmp;
  private CANSparkMax m_PivotArmLeftLeader, m_PivotArmRightFollower;

  private RelativeEncoder shooterEncoderTop, shooterEncoderAmp, PivotArmEncoder;
  private SparkPIDController shooterPIDController, shooterAmpPIDController, PivotArmPIDController;

  private double targetSpeed = 0; 
  private double pivotTargetSpeed = 0;
  private double currentArmPosition;
  private double setPoint;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorTopLeader = new CANSparkMax(ShooterConstants.kShooterMotorTopID, MotorType.kBrushless);
    m_shooterMotorTopLeader.setIdleMode(IdleMode.kCoast);
    m_shooterMotorTopLeader.setInverted(false);

    shooterEncoderTop = m_shooterMotorTopLeader.getEncoder();
    
    shooterPIDController = m_shooterMotorTopLeader.getPIDController();
    shooterPIDController.setP(ShooterConstants.kP);
    shooterPIDController.setI(ShooterConstants.kI);
    shooterPIDController.setD(ShooterConstants.kD);
    shooterPIDController.setFF(ShooterConstants.kFF);
    shooterPIDController.setIZone(ShooterConstants.kIZone);
    shooterPIDController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax);

    m_shooterMotorBottomFollower = new CANSparkMax(ShooterConstants.kShooterMotorBottomID, MotorType.kBrushless);
    m_shooterMotorBottomFollower.setIdleMode(IdleMode.kCoast);
    m_shooterMotorBottomFollower.follow(m_shooterMotorTopLeader, true);

    m_shooterMotorAmp = new CANSparkMax(ShooterConstants.kShooterMotorAmpID, MotorType.kBrushless);
    m_shooterMotorAmp.setIdleMode(IdleMode.kCoast);
    m_shooterMotorAmp.setInverted(false);

    shooterEncoderAmp = m_shooterMotorAmp.getEncoder();
    
    shooterAmpPIDController = m_shooterMotorAmp.getPIDController();
    shooterAmpPIDController.setP(ShooterConstants.kP);
    shooterAmpPIDController.setI(ShooterConstants.kI);
    shooterAmpPIDController.setD(ShooterConstants.kD);
    shooterAmpPIDController.setFF(ShooterConstants.kFF);
    shooterAmpPIDController.setIZone(ShooterConstants.kIZone);
    shooterAmpPIDController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax);

    //now create arm master stuff
    m_PivotArmLeftLeader = new CANSparkMax(ArmConstants.kArmMaster1MotorID, MotorType.kBrushless);
    m_PivotArmLeftLeader.setIdleMode(IdleMode.kCoast);
    m_PivotArmLeftLeader.setInverted(false);

    PivotArmEncoder = m_PivotArmLeftLeader.getEncoder();

    PivotArmPIDController = m_PivotArmLeftLeader.getPIDController();
    PivotArmPIDController.setP(ShooterConstants.kP);
    PivotArmPIDController.setI(ShooterConstants.kI);
    PivotArmPIDController.setD(ShooterConstants.kD);
    PivotArmPIDController.setFF(ShooterConstants.kFF);
    PivotArmPIDController.setIZone(ShooterConstants.kIZone);
    PivotArmPIDController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax);

    //now create arm slave stuff
    
    m_PivotArmRightFollower = new CANSparkMax(ArmConstants.kArmSlaveMotorID, MotorType.kBrushless);
    m_PivotArmRightFollower.setIdleMode(IdleMode.kCoast);
    m_PivotArmRightFollower.follow(m_PivotArmLeftLeader, false);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeedSpeaker(double speed) {
    shooterAmpPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity); 
    shooterPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void setShooterSpeedAmp(double speed){
    shooterAmpPIDController.setReference(-speed, CANSparkMax.ControlType.kVelocity);
    shooterPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void off(){
    shooterAmpPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    shooterPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
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
    double currentArmPosition = PivotArmEncoder.getPosition();
    return (setPoint - currentArmPosition);
  }

  public void setAngle(double angle) {
    PivotArmPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);
  }

  public void sendToDashboard() {
    SmartDashboard.putNumber("displacement values", findDisplacement(ArmConstants.kSetPoint1));
  }

}
