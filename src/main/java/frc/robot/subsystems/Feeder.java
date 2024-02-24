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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.CurrentLimit;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.drive.Drivetrain;
// import frc.robot.subsystems.PivotArm;
import frc.robot.utilities.PidHelper;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotor;
  private RelativeEncoder feederEncoder;
  private SparkPIDController feederPIDController;
  private DigitalInput feederLowNotePresent,feederHighNotePresent;
  private Drivetrain m_drive;
  private PivotArm m_pivotArm;

  public Feeder(Drivetrain drive, PivotArm pivotArm) {
    m_drive = drive;
    m_pivotArm = pivotArm;

    feederLowNotePresent = new DigitalInput(1);
    feederHighNotePresent = new DigitalInput(2);

    //Feeder motor 1
    feederMotor = new CANSparkMax(FeederConstants.kFeederStage1MotorID, MotorType.kBrushless);
    feederMotor.setIdleMode(IdleMode.kBrake);
    feederMotor.setSmartCurrentLimit(CurrentLimit.kFeederAmps);
    feederMotor.setInverted(false);
    
    feederEncoder = feederMotor.getEncoder();
    feederPIDController = feederMotor.getPIDController();
    PidHelper.setupPIDController(this.getName()+"feederStage1PIDController", feederPIDController, FeederConstants.kFeederStage1PIDList);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void feederAutomatic() {
    if(m_pivotArm.isArmAtPosition() && m_drive.isReadyToShoot()) {
      runForward();
    }
    else {
      feedNotesToStow();
    }
  }

  public void feedNotesToStow() {
      if (!hasNote()){
        feederPIDController.setReference(FeederConstants.kFeederLoadRPM, CANSparkMax.ControlType.kVelocity);
      }
      else{ 
        setFeeder2Distance(); 
        stop();
      }
  }

  public void runForward(){
    feederPIDController.setReference(FeederConstants.kFeederShootRPM, CANSparkMax.ControlType.kVelocity);
  }

  public void runReverse(){
    feederPIDController.setReference(-FeederConstants.kFeederLoadRPM, CANSparkMax.ControlType.kVelocity);
  }

  public void stop(){
    // Cuts power off from the motors. Unsure if setVoltage or setReference is better for this purpose.
    feederMotor.setVoltage(0);
    // feederStage1PIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void setFeeder2Distance(){
    feederEncoder.setPosition(0);
    feederPIDController.setReference(5, ControlType.kPosition);

  }

  public boolean hasNote() {
    return feederHighNotePresent.get();
  }

  public boolean has2Notes() {
    return (feederLowNotePresent.get() && feederHighNotePresent.get());
  }

  public boolean isEmpty() {
    return !(feederLowNotePresent.get() || feederHighNotePresent.get());
  }

  public void sendToDashboard() {
    String topic = new String("/"+this.getName()+"/");
		SmartDashboard.putBoolean(topic+"feeder1NotePresent", feederLowNotePresent.get());
    SmartDashboard.putBoolean(topic+"feeder2NotePresent", feederHighNotePresent.get());
    SmartDashboard.putNumber(topic+"Feeder Encoder", feederEncoder.getPosition());
    SmartDashboard.putBoolean(topic+"feederIsEmpty", isEmpty());
    SmartDashboard.putNumber(topic+"Feeder 1 RPM", feederEncoder.getVelocity());  
	}
}
