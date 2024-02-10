package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class ShooterSimulation {

  private final DCMotor shooterGearbox = DCMotor.getNEO(1);

  private CANSparkMax m_shooterMotorTopLeader, m_shooterMotorBottomFollower, m_shooterMotorAmp;
  private RelativeEncoder m_shooterEncoderTop, m_shooterEncoderBottom, m_shooterEncoderAmp;

  private final Field2d m_shooterTrajectorySim;
  Trajectory t = null;

  Shooter m_shooter;
  // DCMotorSim help us simulate what's going on
  private final DCMotorSim m_shooterMotorTopLeaderSim = new DCMotorSim(
    shooterGearbox,
    ShooterConstants.kShooterGearRatio,
    .000005);
  private final DCMotorSim m_shooterMotorBottomFollowerSim = new DCMotorSim(
    shooterGearbox,
    ShooterConstants.kShooterGearRatio,
    .000005);
  private final DCMotorSim m_shooterMotorAmpSim = new DCMotorSim(
    shooterGearbox,
    ShooterConstants.kShooterGearRatio,
    .000005);


  // Create a Mechanism2d visualization of the elevator
  private final double screenWidth = 2.00; // m
  private final double screenHeight = 2.00; // m
  
  private final Mechanism2d m_mech2d = new Mechanism2d(screenWidth, screenHeight);
  private final MechanismRoot2d m_shooterMotorTopLeaderMech2dRoot = m_mech2d.getRoot("m_frontIntakeMech2dRoot", 1, 1.5);
  private final MechanismRoot2d m_shooterMotorBottomFollowerMech2dRoot = m_mech2d.getRoot("m_backIntakeMech2dRoot", 1, .5);
  private final MechanismRoot2d m_shooterMotorAmpMech2dRoot = m_mech2d.getRoot("m_backIntakeMech2dRoot", 1, .5);
  
  private final MechanismLigament2d m_shooterMotorTopLeaderMech2d = m_shooterMotorTopLeaderMech2dRoot
      .append(new MechanismLigament2d("TopLeader",
          0.25,0, 5, new Color8Bit(Color.kRed)));
  
  private final MechanismLigament2d m_shooterMotorBottomFollowerMech2d = m_shooterMotorBottomFollowerMech2dRoot
      .append(new MechanismLigament2d("BottomFollower",
          0.25, 0, 5, new Color8Bit(Color.kRed)));
  
  private final MechanismLigament2d m_shooterMotorAmpMech2d = m_shooterMotorAmpMech2dRoot
      .append(new MechanismLigament2d("AmpMotor",
          0.25, 0, 5, new Color8Bit(Color.kRed)));

  public ShooterSimulation(Shooter shooter, CANSparkMax shooterMotorTopLeader, CANSparkMax shooterMotorBottomFollower, CANSparkMax shooterMotorAmp) {
    m_shooter = shooter;
    m_shooterMotorTopLeader = shooterMotorTopLeader;
    m_shooterMotorBottomFollower = shooterMotorBottomFollower;
    m_shooterMotorAmp = shooterMotorAmp;
    

    m_shooterEncoderTop = m_shooterMotorTopLeader.getEncoder();
    m_shooterEncoderBottom = m_shooterMotorBottomFollower.getEncoder();
    m_shooterEncoderAmp = m_shooterMotorAmp.getEncoder();
    

    REVPhysicsSim.getInstance().addSparkMax(m_shooterMotorTopLeader, shooterGearbox);
    REVPhysicsSim.getInstance().addSparkMax(m_shooterMotorBottomFollower, shooterGearbox);
    REVPhysicsSim.getInstance().addSparkMax(m_shooterMotorAmp, shooterGearbox);

    m_shooterTrajectorySim = new Field2d(); // Screen Width 3.885m and Screen Height 2.000m

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator Sim in the simulator, select Network Tables ->
    // SmartDashboard ->
    // Elevator Sim
    SmartDashboard.putData("Shooter Sim", m_mech2d);
    SmartDashboard.putData("Shooter Trajectory Sim", m_shooterTrajectorySim);

    // m_frontEncoder.setPosition(0.5);
    // m_backEncoder.setPosition(0.5);
    // m_leftEncoder.setPosition(0.5);
    // m_rightEncoder.setPosition(0.5);
  }

  // Boundary Lines
  // private final MechanismRoot2d m_leftBoundStart =
  // m_mech2d.getRoot("m_leftBoundStart", m_OriginX-ShooterConstants.kLeftBound,
  // 0);
  // private final MechanismLigament2d m_leftBoundEnd =
  // m_leftBoundStart.append( new MechanismLigament2d("ElevatorLift",
  // ShooterConstants.kUpperBound, 90,5,new Color8Bit(Color.kRed)));

  public void update() {
    REVPhysicsSim.getInstance().run();

    // Update elevator visualization with simulated position
    m_shooterMotorTopLeaderMech2d.setAngle(
      getUpdatedMotorPosition(m_shooterMotorTopLeaderSim, m_shooterMotorTopLeader, m_shooterEncoderTop));
    m_shooterMotorBottomFollowerMech2d.setAngle(
      getUpdatedMotorPosition(m_shooterMotorBottomFollowerSim, m_shooterMotorBottomFollower, m_shooterEncoderBottom));
    m_shooterMotorAmpMech2d.setAngle(
      getUpdatedMotorPosition(m_shooterMotorAmpSim, m_shooterMotorAmp, m_shooterEncoderAmp));
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