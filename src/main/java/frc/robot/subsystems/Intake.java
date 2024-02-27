package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.commands.intake.IntakeAllIn;
import frc.robot.Constants.CurrentLimit;
import frc.robot.subsystems.sim.IntakeSimulation;
// import frc.robot.subsystems.Feeder;
import frc.robot.utilities.PidHelper;

public class Intake extends SubsystemBase {
	String topic = new String(this.getName()+"/");
	String intakeDirection;

	CANSparkMax m_intakeMotorRight, m_intakeMotorLeft, m_intakeMotorFront, m_intakeMotorBack;
	SparkPIDController frontSparkPIDController, backSparkPIDController, rightSparkPIDController, leftSparkPIDController;
	
	Feeder m_feeder;

	// private RelativeEncoder m_intakeEncoder;
	// private boolean isReversed = false;
	// private DigitalInput m_dIOSensor;

  	IntakeSimulation m_intakeSim;

	public Intake(Feeder feeder) {
		m_feeder = feeder;

        m_intakeMotorRight = new CANSparkMax(IntakeConstants.kIntakeMotorRightID, MotorType.kBrushless);
        m_intakeMotorLeft = new CANSparkMax(IntakeConstants.kIntakeMotorLeftID, MotorType.kBrushless);
        m_intakeMotorFront = new CANSparkMax(IntakeConstants.kIntakeMotorFrontID, MotorType.kBrushless);
        m_intakeMotorBack = new CANSparkMax(IntakeConstants.kIntakeMotorBackID, MotorType.kBrushless);

		m_intakeMotorRight.setSmartCurrentLimit(CurrentLimit.kIntakeAmps);
		m_intakeMotorRight.setInverted(false);
		m_intakeMotorRight.setIdleMode(IdleMode.kCoast);

		m_intakeMotorLeft.setSmartCurrentLimit(CurrentLimit.kIntakeAmps);
		m_intakeMotorLeft.setInverted(false);
		m_intakeMotorLeft.setIdleMode(IdleMode.kCoast);

		m_intakeMotorFront.setSmartCurrentLimit(CurrentLimit.kIntakeAmps);
		m_intakeMotorFront.setInverted(false);
		m_intakeMotorFront.setIdleMode(IdleMode.kCoast);

		m_intakeMotorBack.setSmartCurrentLimit(CurrentLimit.kIntakeAmps);
		m_intakeMotorBack.setInverted(false);
		m_intakeMotorBack.setIdleMode(IdleMode.kCoast);
		intakeDirection = "";

		frontSparkPIDController = m_intakeMotorFront.getPIDController();
		backSparkPIDController = m_intakeMotorBack.getPIDController();
		rightSparkPIDController = m_intakeMotorRight.getPIDController();
		leftSparkPIDController = m_intakeMotorLeft.getPIDController();

		PidHelper.setupPIDController(this.getName()+"frontSparkPIDController", frontSparkPIDController, IntakeConstants.kIntakePIDList);
		PidHelper.setupPIDController(this.getName()+"backSparkPIDController", backSparkPIDController, IntakeConstants.kIntakePIDList);
		PidHelper.setupPIDController(this.getName()+"leftSparkPIDController", leftSparkPIDController, IntakeConstants.kIntakePIDList);
		PidHelper.setupPIDController(this.getName()+"rightSparkPIDController", rightSparkPIDController, IntakeConstants.kIntakePIDList);
	}

	@Override
	public void periodic() {
	  // This method will be called once per scheduler run
	}

	public void intakeAll() {
		frontSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		backSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		rightSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		leftSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		intakeDirection = "All On";
		// isReversed = false;
	}

	public void intakeOff() {
		m_intakeMotorFront.setVoltage(0);
		m_intakeMotorBack.setVoltage(0);
		m_intakeMotorLeft.setVoltage(0);
		m_intakeMotorRight.setVoltage(0);
		// frontSparkPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
		// backSparkPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
		// rightSparkPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
		// leftSparkPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
		intakeDirection = "All Off";

	}

	public void intakeReverse() {
		frontSparkPIDController.setReference(-IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		backSparkPIDController.setReference(-IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		rightSparkPIDController.setReference(-IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		leftSparkPIDController.setReference(-IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		intakeDirection = "All Reverse";

		// isReversed = true;
	}

	public void ejectFront() {
		frontSparkPIDController.setReference(-IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		backSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		rightSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		leftSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		intakeDirection = "eject Front";

	}

	public void ejectBack() {
		frontSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		backSparkPIDController.setReference(-IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		rightSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		leftSparkPIDController.setReference(IntakeConstants.kIntakeRPM, CANSparkMax.ControlType.kVelocity);
		intakeDirection = "eject Back";
	}

	public void intakeAutomatic() {
		if(!m_feeder.hasNoteHigher()) {
			intakeAll();
		}
		else {
			intakeOff();
		}
	}

	public void simulationInit() {
	  	//Setup the simulation
   		m_intakeSim = new IntakeSimulation(this, m_intakeMotorFront, m_intakeMotorBack, m_intakeMotorLeft, m_intakeMotorRight);
	}

	public void simulationPeriodic() {
		//Update elevator simulation
		m_intakeSim.update();
	}

	public void sendToDashboard() {
		String topic = new String(this.getName()+"/");
		SmartDashboard.putNumber(topic+"Front Amps", m_intakeMotorFront.getOutputCurrent());
		SmartDashboard.putNumber(topic+"Back Amps", m_intakeMotorBack.getOutputCurrent());
		SmartDashboard.putNumber(topic+"Left Amps", m_intakeMotorLeft.getOutputCurrent());
		SmartDashboard.putNumber(topic+"Right Amps", m_intakeMotorRight.getOutputCurrent());

		// SmartDashboard.putNumber(topic+"Intake Front Speed", m_intakeMotorFront.getEncoder().getVelocity());
		// SmartDashboard.putNumber(topic+"Intake Back Speed", m_intakeMotorBack.getEncoder().getVelocity());
		// SmartDashboard.putNumber(topic+"Intake Left Speed", m_intakeMotorLeft.getEncoder().getVelocity());
		// SmartDashboard.putNumber(topic+"Intake Right Speed", m_intakeMotorRight.getEncoder().getVelocity());
		// SmartDashboard.putNumber(topic+"intake Position", getRollerPosition());
		// SmartDashboard.putString(topic+"intake Direction", intakeDirection);
	}
}
