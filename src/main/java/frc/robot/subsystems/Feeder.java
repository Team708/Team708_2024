// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.FeederConstants;
import frc.robot.utilities.Helper;

public class Feeder extends SubsystemBase {
  private CANSparkMax m_FeederStage1Motor, m_FeederStage2Motor;
  private RelativeEncoder feederStage1Encoder, feederStage2Encoder;
  private SparkPIDController feederStage1PIDController, feederStage2PIDController;
  private DigitalInput feeder1NotePresent,feeder2NotePresent;

  public Feeder() {
    feeder1NotePresent = new DigitalInput(1);
    feeder2NotePresent = new DigitalInput(2);

    //Feeder motor 1
    m_FeederStage1Motor = new CANSparkMax(FeederConstants.kFeederStage1MotorID, MotorType.kBrushless);
    m_FeederStage1Motor.setIdleMode(IdleMode.kBrake);
    m_FeederStage1Motor.setSmartCurrentLimit(CurrentLimit.kFeederAmps);
    m_FeederStage1Motor.setInverted(false);
    
    feederStage1Encoder = m_FeederStage1Motor.getEncoder();
    feederStage1PIDController = m_FeederStage1Motor.getPIDController();
    Helper.setupPIDController(this.getName()+"feederStage1PIDController", feederStage1PIDController, FeederConstants.kFeederStage1PIDList);

    //Feeder motor 2
    m_FeederStage2Motor = new CANSparkMax(FeederConstants.kFeederStage2MotorID, MotorType.kBrushless);
    m_FeederStage2Motor.setIdleMode(IdleMode.kBrake);
    m_FeederStage1Motor.setSmartCurrentLimit(CurrentLimit.kFeederAmps);
    m_FeederStage2Motor.setInverted(false);
    
    feederStage2Encoder = m_FeederStage2Motor.getEncoder();

    feederStage2PIDController = m_FeederStage2Motor.getPIDController();
    Helper.setupPIDController(this.getName()+"feederStage2PIDController", feederStage2PIDController, FeederConstants.kFeederStage2PIDList);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public void feedNotesToStow(boolean enable){
    if (enable){
      if (!feeder2NotePresent.get()){
        feederStage1PIDController.setReference(FeederConstants.kFeederLoadRPM, CANSparkMax.ControlType.kVelocity);
        feederStage2PIDController.setReference(FeederConstants.kFeederLoadRPM, CANSparkMax.ControlType.kVelocity);
      }else{ 
       stop();
      }
    }else{
      setFeeder2Distance();
      stop();
    }
  }

  public void runForward(double speed){
    feederStage1PIDController.setReference(FeederConstants.kFeederShootRPM, CANSparkMax.ControlType.kVelocity);
    feederStage2PIDController.setReference(FeederConstants.kFeederShootRPM, CANSparkMax.ControlType.kVelocity);
  }

  public void runReverse(){
    feederStage1PIDController.setReference(-FeederConstants.kFeederLoadRPM, CANSparkMax.ControlType.kVelocity);
    feederStage2PIDController.setReference(-FeederConstants.kFeederLoadRPM, CANSparkMax.ControlType.kVelocity);
  }

  public void stop(){
    feederStage1PIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    feederStage2PIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void setFeeder2Distance(){
    feederStage2Encoder.setPosition(0);
    feederStage2PIDController.setReference(5, ControlType.kPosition);

  }

  public boolean hasNote() {
    return feeder2NotePresent.get();
  }

  public boolean has2Notes() {
    return (feeder1NotePresent.get() && feeder2NotePresent.get());
  }

  public boolean isEmpty() {
    return !(feeder1NotePresent.get() || feeder2NotePresent.get());
  }

  public void sendToDashboard() {
    String topic = new String("/"+this.getName()+"/");
		SmartDashboard.putBoolean(topic+"feeder1NotePresent", feeder1NotePresent.get());
    SmartDashboard.putBoolean(topic+"feeder2NotePresent", feeder2NotePresent.get());
    SmartDashboard.putNumber(topic+"Feeder Encoder", feederStage1Encoder.getPosition());
    SmartDashboard.putBoolean(topic+"feederIsEmpty", isEmpty());
    SmartDashboard.putNumber(topic+"Feeder 1 RPM", feederStage1Encoder.getVelocity());  
    SmartDashboard.putNumber(topic+"Feeder 2 RPM", feederStage2Encoder.getVelocity());
	}
}
