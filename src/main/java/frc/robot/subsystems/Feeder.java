// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.utilities.Helper;

public class Feeder extends SubsystemBase {
  private CANSparkMax m_FeederStage1Motor, m_FeederStage2Motor;
  private RelativeEncoder FeederStage1Encoder, FeederStage2Encoder;
  private SparkPIDController FeederStage1PIDController, FeederStage2PIDController;

  public Feeder() {
    //Feeder motor 1
    m_FeederStage1Motor = new CANSparkMax(FeederConstants.kFeederStage1MotorID, MotorType.kBrushless);
    m_FeederStage1Motor.setIdleMode(IdleMode.kBrake);
    m_FeederStage1Motor.setInverted(false);
    
    FeederStage1Encoder = m_FeederStage1Motor.getEncoder();
    
    FeederStage1PIDController = m_FeederStage1Motor.getPIDController();
    Helper.setupPIDController(FeederStage1PIDController, FeederConstants.kFeederStage1PIDList);

    //Feeder motor 2
    m_FeederStage2Motor = new CANSparkMax(FeederConstants.kFeederStage2MotorID, MotorType.kBrushless);
    m_FeederStage2Motor.setIdleMode(IdleMode.kBrake);
    m_FeederStage2Motor.setInverted(false);
    
    FeederStage2Encoder = m_FeederStage2Motor.getEncoder();
    
    FeederStage2PIDController = m_FeederStage2Motor.getPIDController();
    Helper.setupPIDController(FeederStage2PIDController, FeederConstants.kFeederStage2PIDList);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void feederThroughStage1() {
    m_FeederStage1Motor.set(1);
  }

  //ultimately check if sensor input is true for whether note has passed through stage 2
  public void feederThroughStage2() {
    m_FeederStage2Motor.set(1.0);
  }


}
