// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utilities.PidHelper;
import frc.robot.utilities.MathUtils;
import frc.robot.OI;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.MathUtil;

//import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;


public class PivotArm extends SubsystemBase {
  private CANSparkMax m_PivotArmLeftLeader, m_PivotArmRightFollower;
  private RelativeEncoder PivotArmEncoder;
  private DutyCycleEncoder absEncoder;
  private SparkPIDController pivotArmPIDController;
  private double targetArmAngle;
  private SparkLimitSwitch forwardLimit, reverseLimit;
  private Drivetrain m_drive;

  private double distance;
  private InterpolatingDoubleTreeMap interpolatingTreeMap = new InterpolatingDoubleTreeMap();


  public PivotArm(Drivetrain drive) {
    m_drive = drive;

    absEncoder = new DutyCycleEncoder(0);
    absEncoder.reset();
    
    //Leader arm motor
    m_PivotArmLeftLeader = new CANSparkMax(ArmConstants.kArmMasterMotorID, MotorType.kBrushless);
    m_PivotArmLeftLeader.setIdleMode(IdleMode.kBrake);
    m_PivotArmLeftLeader.setSmartCurrentLimit(CurrentLimit.kArmAmps);
    m_PivotArmLeftLeader.setInverted(false);
    
    forwardLimit = m_PivotArmLeftLeader.getForwardLimitSwitch(Type.kNormallyOpen);
    forwardLimit.enableLimitSwitch(true);
    reverseLimit = m_PivotArmLeftLeader.getReverseLimitSwitch(Type.kNormallyOpen);
    reverseLimit.enableLimitSwitch(true);
    
    PivotArmEncoder = m_PivotArmLeftLeader.getEncoder();
    PivotArmEncoder.setPositionConversionFactor(ArmConstants.kPivotArmGearRatio);
    PivotArmEncoder.setPosition(getAbsolutePosition());
    
    pivotArmPIDController = m_PivotArmLeftLeader.getPIDController();
    PidHelper.setupPIDController(this.getName()+"pivotArmPIDController", pivotArmPIDController, ArmConstants.kPivotArmPIDList);
    
    //Follower arm motor
    m_PivotArmRightFollower = new CANSparkMax(ArmConstants.kArmSlaveMotorID, MotorType.kBrushless);
    m_PivotArmRightFollower.setIdleMode(IdleMode.kBrake);
    m_PivotArmRightFollower.setSmartCurrentLimit(40);
    m_PivotArmRightFollower.follow(m_PivotArmLeftLeader, true);

    //interpolation stuff
    // map = new TreeMap<Double, Double>();
    // map.put(1.3, 52.0);
    // map.put(2.0, 40.0);
    // map.put(2.77, 32.0);
    // map.put(3.6, 27.0);
    // map.put(4.75, 22.0);
    // map.put(6.0,21.0);

    interpolatingTreeMap.put(1.3, 52.0);
    interpolatingTreeMap.put(2.0, 40.0);
    interpolatingTreeMap.put(2.77, 32.0);
    interpolatingTreeMap.put(3.6, 27.0);
    interpolatingTreeMap.put(4.75, 22.0);
    interpolatingTreeMap.put(6.0,21.0);
  }
  
  @Override
  public void periodic() {
    
  }
  
  //current and voltage limits
  
  public double getPosition() {
    return PivotArmEncoder.getPosition();
  }
  
  public double getAbsolutePosition() {
    return (-ArmConstants.kArmClockingOffset-(absEncoder.getAbsolutePosition()*ArmConstants.kArmScalingFactor)-ArmConstants.kArmAbsEncoderOffset);
  }
  public double inputTransform(double input) {
        //return MathUtils.singedSquare(MathUtils.applyDeadband(input));
        return MathUtils.cubicLinear(MathUtils.applyDeadband(input), 0.9, 0.1);
    }
  public void operateByController() {
    double desiredY = -inputTransform(OI.getOperatorLeftY());
    m_PivotArmLeftLeader.set(desiredY);

  }
  //Determine units for arm as it's not completely tested
  public void setArmAngle(double angle) {
    targetArmAngle = angle;
    pivotArmPIDController.setReference(angle, CANSparkBase.ControlType.kPosition);    
  }
  
  // public double findArmAngle () {
  //   double distance = m_drive.getDistanceToTarget();
  //   if(distance < ArmConstants.kMaxShootingDistance) {
  //     //This is where we add the equations to solve for targetArmAngle
  //     targetArmAngle = 1.86876*Math.pow(distance, 2) -19.9052*distance + 73.639; //Using constant temporarily
  //     return targetArmAngle;
  //   }
  //   else {
  //     return 0;
  //   }
  // }

  // public double findArmAngle () {
  //   //This is where we add the equations to solve for targetArmAn
 
  //   distance = m_drive.getDistanceToTarget();
  //   if(distance < ArmConstants.kMaxShootingDistance){
  //       // if(!map.containsKey(distance)){
  //         floorKey = 1.3;//map.floorKey(distance);
  //         ceilingKey = 2.0;//map.ceilingKey(distance);

  //         topElem = map.get(ceilingKey);
  //         bottomElem = map.get(floorKey);
 
  //         //Interpolatable interpolateBetweenKeys = new Interpolatable<Double>();
          
  //         double val = MathUtil.interpolate(bottomElem, topElem, MathUtil.inverseInterpolate(bottomElem, topElem, distance));
  //         SmartDashboard.putNumber("val angle", val);
  //         return val;
  //         //return 0;
  //       // }
  //       // else{
  //       //   return map.get(distance);
  //       // }
  //   }
  //   return 0;
 
   
  //   //targetArmAngle = ArmConstants.kBumperShotAngle; //Using constant temporarily
   
  //   //return targetArmAngle;
  // }

  public double findArmAngle(){
    distance = m_drive.getDistanceToTarget();
    if(distance < ArmConstants.kMaxShootingDistance){
      return interpolatingTreeMap.get(distance);
    }
    return 0;
  }


  public boolean isArmAtPosition() {
    return (targetArmAngle- getPosition()) < ArmConstants.kThresholdArm;
  }
  

  public void sendToDashboard() {
    SmartDashboard.putNumber("Arm Encoder Position", getPosition());
    SmartDashboard.putNumber("Absolute Encoder Position", getAbsolutePosition());
    SmartDashboard.putBoolean("Arm Forward Limit", forwardLimit.isPressed());
    SmartDashboard.putBoolean("Arm Reverse Limit", reverseLimit.isPressed());
    SmartDashboard.putNumber("Calculate Arm angle", findArmAngle());
    SmartDashboard.putNumber("Upper bound", topElem);
    SmartDashboard.putNumber("Lower bound", bottomElem);
  }
}
