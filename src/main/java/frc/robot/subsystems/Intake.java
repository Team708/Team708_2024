package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CurrentLimit;

public class Intake extends SubsystemBase{
    
    private CANSparkMax m_intakeMotorRight;
    private CANSparkMax m_intakeMotorLeft;
    private CANSparkMax m_intakeMotorFront;
    private CANSparkMax m_intakeMotorBack;

    // private RelativeEncoder m_intakeEncoder;
    
    // private boolean isReversed = false;

    // private DigitalInput m_dIOSensor;

    public Intake(DigitalInput m_dIOSensor){

        // this.m_dIOSensor = m_dIOSensor;

        m_intakeMotorRight = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_intakeMotorLeft = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_intakeMotorFront = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
        m_intakeMotorBack = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

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
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void intakeAll(){
        m_intakeMotorRight.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorLeft.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorFront.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorBack.set(IntakeConstants.kIntakeSpeed);
        // isReversed = false;
    }

    public void intakeOff(){
        m_intakeMotorRight.set(0);
        m_intakeMotorLeft.set(0);
        m_intakeMotorFront.set(0);
        m_intakeMotorBack.set(0);
    }

    public void intakeReverse(){
        m_intakeMotorRight.set(-IntakeConstants.kIntakeSpeed);
        m_intakeMotorLeft.set(-IntakeConstants.kIntakeSpeed);
        m_intakeMotorFront.set(-IntakeConstants.kIntakeSpeed);
        m_intakeMotorBack.set(-IntakeConstants.kIntakeSpeed);
        // isReversed = true;
    }

    public void ejectFront(){
        m_intakeMotorRight.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorLeft.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorFront.set(-IntakeConstants.kIntakeSpeed);
        m_intakeMotorBack.set(IntakeConstants.kIntakeSpeed);
    }

    public void ejectBack(){
        m_intakeMotorRight.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorLeft.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorFront.set(IntakeConstants.kIntakeSpeed);
        m_intakeMotorBack.set(-IntakeConstants.kIntakeSpeed);
    }


    // public boolean sensorDetected(){
    // return !m_dIOSensor.get();
    // }

    public void sendToDashboard(){
        // SmartDashboard.putNumber("intake speed", getRollerSpeed());
        // SmartDashboard.putNumber("intake Position", getRollerPosition());
    }

}
