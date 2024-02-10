package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CurrentLimit;
import frc.robot.subsystems.sim.IntakeSimulation;

public class Intake extends SubsystemBase{
	
	private CANSparkMax m_intakeMotorRight;
	private CANSparkMax m_intakeMotorLeft;
	private CANSparkMax m_intakeMotorFront;
	private CANSparkMax m_intakeMotorBack;

	private String intakeDirection;

	// private RelativeEncoder m_intakeEncoder;
	
	// private boolean isReversed = false;

	// private DigitalInput m_dIOSensor;

  IntakeSimulation m_intakeSim;

	public Intake(){

		// this.m_dIOSensor = m_dIOSensor;

		m_intakeMotorRight = new CANSparkMax(IntakeConstants.kIntakeMotorRightID, MotorType.kBrushless);
		m_intakeMotorLeft = new CANSparkMax(IntakeConstants.kIntakeMotorLeftID, MotorType.kBrushless);
		m_intakeMotorFront = new CANSparkMax(IntakeConstants.kIntakeMotorFrontID, MotorType.kBrushless);
		m_intakeMotorBack = new CANSparkMax(IntakeConstants.kIntakeMotorBackID, MotorType.kBrushless);

		m_intakeMotorRight.setSmartCurrentLimit(CurrentLimit.kIntake);
		m_intakeMotorRight.setInverted(false);
		m_intakeMotorRight.setIdleMode(IdleMode.kCoast);

		m_intakeMotorLeft.setSmartCurrentLimit(CurrentLimit.kIntake);
		m_intakeMotorLeft.setInverted(false);
		m_intakeMotorLeft.setIdleMode(IdleMode.kCoast);

		m_intakeMotorFront.setSmartCurrentLimit(CurrentLimit.kIntake);
		m_intakeMotorFront.setInverted(false);
		m_intakeMotorFront.setIdleMode(IdleMode.kCoast);

		m_intakeMotorBack.setSmartCurrentLimit(CurrentLimit.kIntake);
		m_intakeMotorBack.setInverted(false);
		m_intakeMotorBack.setIdleMode(IdleMode.kCoast);
		intakeDirection = "";
	}

	@Override
	public void periodic() {
	  // This method will be called once per scheduler run
	}

	public void intakeAll(){
		m_intakeMotorRight.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorLeft.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorFront.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorBack.setVoltage(IntakeConstants.kIntakeSpeed);
		intakeDirection = "All On";
		// isReversed = false;
	}

	public void intakeOff(){
		m_intakeMotorRight.setVoltage(0);
		m_intakeMotorLeft.setVoltage(0);
		m_intakeMotorFront.setVoltage(0);
		m_intakeMotorBack.setVoltage(0);
		intakeDirection = "All Off";

	}

	public void intakeReverse(){
		m_intakeMotorRight.setVoltage(-IntakeConstants.kIntakeSpeed);
		m_intakeMotorLeft.setVoltage(-IntakeConstants.kIntakeSpeed);
		m_intakeMotorFront.setVoltage(-IntakeConstants.kIntakeSpeed);
		m_intakeMotorBack.setVoltage(-IntakeConstants.kIntakeSpeed);
		intakeDirection = "All Reverse";

		// isReversed = true;
	}

	public void ejectFront(){
		m_intakeMotorRight.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorLeft.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorFront.setVoltage(-IntakeConstants.kIntakeSpeed);
		m_intakeMotorBack.setVoltage(IntakeConstants.kIntakeSpeed);
		intakeDirection = "eject Front";

	}

	public void ejectBack(){
		m_intakeMotorRight.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorLeft.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorFront.setVoltage(IntakeConstants.kIntakeSpeed);
		m_intakeMotorBack.setVoltage(-IntakeConstants.kIntakeSpeed);
		intakeDirection = "eject Back";

	}

	// public boolean sensorDetected(){
	// return !m_dIOSensor.get();
	// }

	public void simulationInit() {
	  //Setup the simulation
    m_intakeSim = new IntakeSimulation(this, m_intakeMotorFront, m_intakeMotorBack, m_intakeMotorLeft, m_intakeMotorRight);
  
	}

	public void simulationPeriodic() {
    //Update elevator simulation
    m_intakeSim.update();
	}

	public void sendToDashboard(){
		// SmartDashboard.putNumber("intake speed", getRollerSpeed());
		// SmartDashboard.putNumber("intake Position", getRollerPosition());
		SmartDashboard.putString("intake Direction", intakeDirection);

	}


}
