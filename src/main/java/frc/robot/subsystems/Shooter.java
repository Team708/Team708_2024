// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utilities.PidHelper;
// import frc.robot.subsystems.ShooterSimulation;


public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotorTopLeader, m_shooterMotorBottomFollower, m_shooterMotorAmp;

  private RelativeEncoder shooterEncoderTop, shooterEncoderBottom, shooterEncoderAmp;
  private SparkPIDController shooterSpeakerPIDController, shooterAmpPIDController;

  private double targetSpeed = 1000;   // Arbitarily high number

  ShooterSimulation m_shootersim;
  
  /** Creates a new Shooter. */
  public Shooter() {
    //Top shooter motor
    m_shooterMotorTopLeader = new CANSparkMax(ShooterConstants.kShooterMotorTopID, MotorType.kBrushless);
    m_shooterMotorTopLeader.setIdleMode(IdleMode.kCoast);
    m_shooterMotorTopLeader.setSmartCurrentLimit(CurrentLimit.kShooterAmps);
    m_shooterMotorTopLeader.setInverted(true);

    shooterEncoderTop = m_shooterMotorTopLeader.getEncoder();
    
    shooterSpeakerPIDController = m_shooterMotorTopLeader.getPIDController();
    PidHelper.setupPIDController(this.getName()+"shooterSpeakerPIDController", shooterSpeakerPIDController, ShooterConstants.kShooterTopPIDList);

    //Bottom shooter motor
    m_shooterMotorBottomFollower = new CANSparkMax(ShooterConstants.kShooterMotorBottomID, MotorType.kBrushless);
    m_shooterMotorBottomFollower.setIdleMode(IdleMode.kCoast);
    m_shooterMotorBottomFollower.setSmartCurrentLimit(CurrentLimit.kShooterAmps);
    m_shooterMotorBottomFollower.follow(m_shooterMotorTopLeader, false);

    shooterEncoderBottom = m_shooterMotorBottomFollower.getEncoder();

    //Amp shooter motor
    m_shooterMotorAmp = new CANSparkMax(ShooterConstants.kShooterMotorAmpID, MotorType.kBrushless);
    m_shooterMotorAmp.setIdleMode(IdleMode.kCoast);
    m_shooterMotorAmp.setSmartCurrentLimit(CurrentLimit.kShooterAmps);
    m_shooterMotorAmp.setInverted(false);

    shooterEncoderAmp = m_shooterMotorAmp.getEncoder();
    
    shooterAmpPIDController = m_shooterMotorAmp.getPIDController();
    PidHelper.setupPIDController(this.getName()+"shooterAmpPIDController", shooterAmpPIDController, ShooterConstants.kShooterAmpPIDList);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeedSpeaker() { 
    targetSpeed = ShooterConstants.kShooterSpeakerRPM;
    shooterAmpPIDController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
    shooterSpeakerPIDController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void setShooterSpeedAmp(){
    targetSpeed = ShooterConstants.kShooterAmpRPM;
    shooterAmpPIDController.setReference(-targetSpeed, CANSparkMax.ControlType.kVelocity);
    shooterSpeakerPIDController.setReference(targetSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void setShooterSpeedReverse() {
    targetSpeed = ShooterConstants.kShooterLowRPM;
    shooterAmpPIDController.setReference(-targetSpeed, CANSparkMax.ControlType.kVelocity);
    shooterSpeakerPIDController.setReference(-targetSpeed, CANSparkMax.ControlType.kVelocity);
  }
  public void off(){
    m_shooterMotorTopLeader.setVoltage(0);
    m_shooterMotorAmp.setVoltage(0);
    // shooterAmpPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    // shooterSpeakerPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public boolean isAtSpeed() {
    if (Math.abs(shooterEncoderTop.getVelocity()) > (targetSpeed) &&
        Math.abs(shooterEncoderBottom.getVelocity()) > (targetSpeed)&&
        Math.abs(shooterEncoderAmp.getVelocity()) > (targetSpeed)){
      return true;
    }
    return false;
  }

  public void sendToDashboard() {

    String topic = new String(this.getName()+"/");
    // SmartDashboard.putNumber(topic+"Shooter Top Velocity", shooterEncoderTop.getVelocity());
    // SmartDashboard.putNumber(topic+"Shooter Bottom Velocity", shooterEncoderBottom.getVelocity());
    // SmartDashboard.putNumber(topic+"Shooter Amp Velocity", shooterEncoderAmp.getVelocity());
    // SmartDashboard.putNumber(topic+"Shooter Bottom Amps", m_shooterMotorTopLeader.getOutputCurrent());
    // SmartDashboard.putNumber(topic+"Shooter Top Amps", m_shooterMotorAmp.getOutputCurrent());
    SmartDashboard.putBoolean("Shooter At Speed", isAtSpeed());

  }
  
  // public void simulationInit() {
	//   //Setup the simulation
  //   m_shootersim = new ShooterSimulation(this, m_shooterMotorTopLeader, m_shooterMotorBottomFollower, m_shooterMotorAmp);
	// }

	// public void simulationPeriodic() {
  //   //Update elevator simulation
  //   m_shootersim.update();
	// }
}
