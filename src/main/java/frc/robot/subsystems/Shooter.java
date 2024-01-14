// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotor;
  public RelativeEncoder shooterEncoder;
  private SparkPIDController shooterPIDController;
  public double targetSpeed = 0;

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorID, MotorType.kBrushless);
    shooterEncoder = m_shooterMotor.getEncoder();

    shooterPIDController = m_shooterMotor.getPIDController();
    shooterPIDController.setP(ShooterConstants.kP);
    shooterPIDController.setI(ShooterConstants.kI);
    shooterPIDController.setD(ShooterConstants.kD);
    shooterPIDController.setFF(ShooterConstants.kFF);
    shooterPIDController.setIZone(ShooterConstants.kIZone);
    shooterPIDController.setOutputRange(ShooterConstants.kMin, ShooterConstants.kMax);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooterOn() {
   m_shooterMotor.set(0.5); 
  }

  public void shooterOff(){
    m_shooterMotor.set(0);
  }

  public void setVelocity(double velocity){
    targetSpeed = velocity;
    shooterPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  public boolean isShooterAtSpeed() {
    // TODO Auto-generated method stub
    if ((Math.abs(shooterEncoder.getVelocity()) > (targetSpeed) * ShooterConstants.kThreshhold)){
      return true;
    }else{
      return false;
    }
  }
}
