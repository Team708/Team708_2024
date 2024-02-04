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
import frc.robot.Constants.FeederConstants;
import frc.robot.utilities.Helper;

public class Shooter extends SubsystemBase {
  private CANSparkMax m_shooterMotorTopLeader, m_shooterMotorBottomFollower, m_shooterMotorAmp;
  private CANSparkMax m_PivotArmLeftLeader, m_PivotArmRightFollower;
  private CANSparkMax m_FeederStage1Motor, m_FeederStage2Motor;

  private RelativeEncoder shooterEncoderTop, shooterEncoderAmp, PivotArmEncoder, FeederStage1Encoder, FeederStage2Encoder;
  private SparkPIDController shooterSpeakerPIDController, shooterAmpPIDController, PivotArmPIDController, FeederStage1PIDController, FeederStage2PIDController;

  private double targetSpeed = 0; 
  private double pivotTargetSpeed = 0;

  /** Creates a new Shooter. */
  public Shooter() {
    //Top shooter motor
    m_shooterMotorTopLeader = new CANSparkMax(ShooterConstants.kShooterMotorTopID, MotorType.kBrushless);
    m_shooterMotorTopLeader.setIdleMode(IdleMode.kCoast);

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

  public void setShooterSpeedSpeaker(double speed) { 
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
    if ((Math.abs(PivotArmEncoder.getVelocity()) > (targetSpeed) * ShooterConstants.kThreshhold)){
      return true;
    }
    return false;
  }
  //Now we want to get arm from current position to target setpoint
  public double findDisplacement(double setPoint) {
    double currentArmPosition = PivotArmEncoder.getPosition();
    return (setPoint - currentArmPosition);
  }

  public void setAngle(double angle) {
    PivotArmPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);
  }
}
