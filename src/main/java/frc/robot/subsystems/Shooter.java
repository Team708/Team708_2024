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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.Helper;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotorTopLeader, m_shooterMotorBottomFollower, m_shooterMotorAmp;

  private RelativeEncoder shooterEncoderTop, shooterEncoderAmp;
  private SparkPIDController shooterSpeakerPIDController, shooterAmpPIDController;

  private double targetSpeed = 0; 

  /** Creates a new Shooter. */
  public Shooter() {
    //Top shooter motor
    m_shooterMotorTopLeader = new CANSparkMax(ShooterConstants.kShooterMotorTopID, MotorType.kBrushless);
    m_shooterMotorTopLeader.setIdleMode(IdleMode.kCoast);
    m_shooterMotorTopLeader.setInverted(false);

    shooterEncoderTop = m_shooterMotorTopLeader.getEncoder();
    
    shooterSpeakerPIDController = m_shooterMotorTopLeader.getPIDController();
    Helper.setupPIDController(shooterSpeakerPIDController, ShooterConstants.kShooterTopPIDList);

    //Bottom shooter motor
    m_shooterMotorBottomFollower = new CANSparkMax(ShooterConstants.kShooterMotorBottomID, MotorType.kBrushless);
    m_shooterMotorBottomFollower.setIdleMode(IdleMode.kCoast);
    m_shooterMotorBottomFollower.follow(m_shooterMotorTopLeader, true);

    //Amp shooter motor
    m_shooterMotorAmp = new CANSparkMax(ShooterConstants.kShooterMotorAmpID, MotorType.kBrushless);
    m_shooterMotorAmp.setIdleMode(IdleMode.kCoast);
    m_shooterMotorAmp.setInverted(false);

    shooterEncoderAmp = m_shooterMotorAmp.getEncoder();
    
    shooterAmpPIDController = m_shooterMotorAmp.getPIDController();
    Helper.setupPIDController(shooterAmpPIDController, ShooterConstants.kShooterAmpPIDList);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeedSpeaker(double speed) { 
    shooterAmpPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    shooterSpeakerPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void setShooterSpeedAmp(double speed){
    shooterAmpPIDController.setReference(-speed, CANSparkMax.ControlType.kVelocity);
    shooterSpeakerPIDController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  public void off(){
    shooterAmpPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    shooterSpeakerPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public boolean isShooterSpeakerAtSpeed() {
    if ((Math.abs(shooterEncoderTop.getVelocity()) > (targetSpeed) * ShooterConstants.kThreshhold)){
      return true;
    }
    return false;
  }

  public boolean isShooterAmpAtSpeed() {
    if ((Math.abs(shooterEncoderAmp.getVelocity()) > (targetSpeed) * ShooterConstants.kThreshhold)){
      return true;
    }
    return false;
  }
}
