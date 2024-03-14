// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
// Library imports
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.OI;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.PidHelper;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private SparkPIDController climberPIDController;
  private double targetDistance;
   
  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new CANSparkMax(ClimberConstants.kClimberMotorID, MotorType.kBrushless);
    climberMotor.setIdleMode(IdleMode.kBrake);
    climberMotor.setSmartCurrentLimit(CurrentLimit.kClimberAmps);
    climberMotor.setInverted(false);

    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPositionConversionFactor(ClimberConstants.kClimberConversionFactor);

    climberPIDController = climberMotor.getPIDController();

    PidHelper.setupPIDController(this.getName()+"climberPIDController", climberPIDController, ClimberConstants.kClimberPIDList);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPosition() {
    return climberEncoder.getPosition();
  }

  //Determine units for arm as it's not completely tested
  public void setDistance(double angle) {
    targetDistance = angle;
    climberPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);    
  }

  public double inputTransform(double input) {
        //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
        return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
    }

  public void climbByController() {
    // double desiredY = -inputTransform(OI.getClimberLeftY());
    // if(Math.abs(desiredY) > ControllerConstants.kClimberDeadBandLeftY) {
    //   setDistance(getPosition()+(desiredY*2));
    // }
    // climberMotor.set(-desiredY);
  }
}
