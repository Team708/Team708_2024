package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Helper {
    private static Helper instance;
    
    /**
     * Returns the Helper instance.
     *
     * @return the instance
     */
    public static synchronized Helper getInstance() {
        if (instance == null) {
            instance = new Helper();
        }
        return instance;
    }

    Helper(){

    }
    
    private static SparkPIDController m_PIDController, previousSelection;
    private static SendableChooser<SparkPIDController> m_ControllerChooser;// = new SendableChooser<>();
    
    private static double[] m_PIDList;
    private static boolean PIDListUpdated = false;

    /**
     * Sets the PID controller values. Still requires the CANSparkMax and the PIDController created beforehand.
     * @param PIDController Name of the controller to assign values to
     * @param PIDList 1 dimensional array of doubles for the 7 PID values. In order, the values are
     * kP, kI, kD, kFF, KIZone, kMin, and kMax
     */
    public static void setupPIDController(String name, SparkPIDController PIDController, double[] PIDList) {
        m_PIDList = PIDList;
        m_PIDController = PIDController;
        
        m_ControllerChooser.addOption(name, m_PIDController);
        setValues();
    }

    public static void setValues(){
        m_PIDController.setP(m_PIDList[0]);
        m_PIDController.setI(m_PIDList[1]);
        m_PIDController.setD(m_PIDList[2]);
        m_PIDController.setFF(m_PIDList[3]);
        m_PIDController.setIZone(m_PIDList[4]);
        m_PIDController.setOutputRange(m_PIDList[5], m_PIDList[6]);
    }

    public void update() {
        m_PIDController = m_ControllerChooser.getSelected();
        if (m_PIDController != null){
            if (m_PIDController != previousSelection){
                System.out.println("3");
                SmartDashboard.putNumber("PID P",m_PIDController.getP());
                SmartDashboard.putNumber("PID I",m_PIDController.getI());
                SmartDashboard.putNumber("PID D",m_PIDController.getD());
                SmartDashboard.putNumber("PID FF",m_PIDController.getFF());
                SmartDashboard.putNumber("PID I Zone",m_PIDController.getIZone());
                SmartDashboard.putNumber("PID Output Range Min",m_PIDController.getOutputMin());
                SmartDashboard.putNumber("PID Output Range Max",m_PIDController.getOutputMax());
                previousSelection = m_PIDController;
            }

            storeTuningVal(0, SmartDashboard.getNumber("PID P",0.0));
            storeTuningVal(1, SmartDashboard.getNumber("PID I",0.0));
            storeTuningVal(2, SmartDashboard.getNumber("PID D",0.0));
            storeTuningVal(3, SmartDashboard.getNumber("PID FF",0.0));
            storeTuningVal(4, SmartDashboard.getNumber("PID I Zone",0.0));
            storeTuningVal(5, SmartDashboard.getNumber("PID Output Range Min",0.0));
            storeTuningVal(6, SmartDashboard.getNumber("PID Output Range Max",0.0));

            if (PIDListUpdated){
                setValues();
                PIDListUpdated = false;
            }
        }
    }

    public static void storeTuningVal(int index, double tuningVal){
        if (m_PIDList[index] != tuningVal){
            m_PIDList[index] = tuningVal;
            PIDListUpdated = true;
        }
    }

    public void addChooser(SendableChooser<SparkPIDController> ControllerChooser){
        m_ControllerChooser = ControllerChooser;
        previousSelection = new CANSparkMax(0, MotorType.kBrushless).getPIDController();
        m_ControllerChooser.setDefaultOption("None" , previousSelection);
        SmartDashboard.putData("SparkMax Chooser", m_ControllerChooser);
        SmartDashboard.putNumber("PID P",0);
        SmartDashboard.putNumber("PID I",0);
        SmartDashboard.putNumber("PID D",0);
        SmartDashboard.putNumber("PID FF",0);
        SmartDashboard.putNumber("PID I Zone",0);
        SmartDashboard.putNumber("PID Output Range Min",0);
        SmartDashboard.putNumber("PID Output Range Max",0);
    }
}
