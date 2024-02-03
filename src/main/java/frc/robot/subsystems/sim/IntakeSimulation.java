// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sim;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeSimulation {

  private final DCMotor intakeGearbox = DCMotor.getNEO(1);

  private CANSparkMax m_frontMotor, m_backMotor, m_leftMotor, m_rightMotor;
  private RelativeEncoder m_frontEncoder, m_backEncoder, m_leftEncoder, m_rightEncoder;

  private final Field2d m_intakeTrajectorySim;
  Trajectory t = null;

  Intake m_intake;

  // DCMotorSim help us simulate what's going on
  private final DCMotorSim m_frontIntakeSim = new DCMotorSim(
    intakeGearbox,
    IntakeConstants.kRollerGearRatio,
    .000005);
  private final DCMotorSim m_backIntakeSim = new DCMotorSim(
    intakeGearbox,
    IntakeConstants.kRollerGearRatio,
    .000005);
  private final DCMotorSim m_leftIntakeSim = new DCMotorSim(
    intakeGearbox,
    IntakeConstants.kRollerGearRatio,
    .000005);
  private final DCMotorSim m_rightIntakeSim = new DCMotorSim(
    intakeGearbox,
    IntakeConstants.kRollerGearRatio,
    .000005);


  // Create a Mechanism2d visualization of the elevator
  private final double screenWidth = 2.00; // m
  private final double screenHeight = 2.00; // m
  
  private final Mechanism2d m_mech2d = new Mechanism2d(screenWidth, screenHeight);
  private final MechanismRoot2d m_frontIntakeMech2dRoot = m_mech2d.getRoot("m_frontIntakeMech2dRoot", 1, 1.5);
  private final MechanismRoot2d m_backIntakeMech2dRoot = m_mech2d.getRoot("m_backIntakeMech2dRoot", 1, .5);
  private final MechanismRoot2d m_leftIntakeMech2dRoot = m_mech2d.getRoot("m_leftIntakeMech2dRoot", 0.5, 1);
  private final MechanismRoot2d m_rightIntakeMech2dRoot = m_mech2d.getRoot("m_rightIntakeMech2dRoot", 1.5, 1);

  private final MechanismLigament2d m_frontIntakeMech2d = m_frontIntakeMech2dRoot
      .append(new MechanismLigament2d("FrontIntake",
          0.25,0, 5, new Color8Bit(Color.kRed)));
  
  private final MechanismLigament2d m_backIntakeMech2d = m_backIntakeMech2dRoot
      .append(new MechanismLigament2d("BackIntake",
          0.25, 0, 5, new Color8Bit(Color.kRed)));
  
  private final MechanismLigament2d m_leftIntakeMech2d = m_leftIntakeMech2dRoot
      .append(new MechanismLigament2d("LeftIntake",
          0.25, 0, 5, new Color8Bit(Color.kRed)));
 
  private final MechanismLigament2d m_rightIntakeMech2d = m_rightIntakeMech2dRoot
      .append(new MechanismLigament2d("RightIntake",
          0.25
          , 0, 5, new Color8Bit(Color.kRed)));

  public IntakeSimulation(Intake intake, CANSparkMax frontMotor, CANSparkMax backMotor, CANSparkMax leftMotor, CANSparkMax rightMotor) {
    m_intake = intake;
    m_frontMotor = frontMotor;
    m_backMotor = backMotor;
    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;

    m_frontEncoder = m_frontMotor.getEncoder();
    m_backEncoder = m_backMotor.getEncoder();
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    REVPhysicsSim.getInstance().addSparkMax(m_frontMotor, intakeGearbox);
    REVPhysicsSim.getInstance().addSparkMax(m_backMotor, intakeGearbox);
    REVPhysicsSim.getInstance().addSparkMax(m_leftMotor, intakeGearbox);
    REVPhysicsSim.getInstance().addSparkMax(m_rightMotor, intakeGearbox);

    m_intakeTrajectorySim = new Field2d(); // Screen Width 3.885m and Screen Height 2.000m

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator Sim in the simulator, select Network Tables ->
    // SmartDashboard ->
    // Elevator Sim
    SmartDashboard.putData("Intake Sim", m_mech2d);
    SmartDashboard.putData("Intake Trajectory Sim", m_intakeTrajectorySim);

    // m_frontEncoder.setPosition(0.5);
    // m_backEncoder.setPosition(0.5);
    // m_leftEncoder.setPosition(0.5);
    // m_rightEncoder.setPosition(0.5);
  }

  // Boundary Lines
  // private final MechanismRoot2d m_leftBoundStart =
  // m_mech2d.getRoot("m_leftBoundStart", m_OriginX-IntakeConstants.kLeftBound,
  // 0);
  // private final MechanismLigament2d m_leftBoundEnd =
  // m_leftBoundStart.append( new MechanismLigament2d("ElevatorLift",
  // IntakeConstants.kUpperBound, 90,5,new Color8Bit(Color.kRed)));

  public void update() {
    REVPhysicsSim.getInstance().run();

    // Update elevator visualization with simulated position
    m_frontIntakeMech2d.setAngle(
      getUpdatedMotorPosition(m_frontIntakeSim, m_frontMotor, m_frontEncoder));
    m_backIntakeMech2d.setAngle(
      getUpdatedMotorPosition(m_backIntakeSim, m_backMotor, m_backEncoder));
    m_leftIntakeMech2d.setAngle(
      getUpdatedMotorPosition(m_leftIntakeSim, m_leftMotor, m_leftEncoder));
    m_rightIntakeMech2d.setAngle(
      getUpdatedMotorPosition(m_rightIntakeSim, m_rightMotor, m_rightEncoder));
  }

  private double getUpdatedMotorPosition(DCMotorSim motorSim, CANSparkMax motor, RelativeEncoder encoder){
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    motorSim.setInput(motor.getAppliedOutput());

    // Next, we update it. The standard loop time is 20ms.
    motorSim.update(GlobalConstants.kLoopTime);

    double position = motorSim.getAngularPositionRad();

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    encoder.setPosition(position);

    return position;
  }
}