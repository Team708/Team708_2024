package frc.robot.utilities;

import com.revrobotics.SparkPIDController;
public class Helper {
    private SparkPIDController PIDController;
    private Chooser<SparkMax> m_ControllerChooser;
    private double[] PIDList;
    private boolean PIDListUpdated;

    public helper(){
        PIDListUpdated = false;
    }
    /**
     * Sets the PID controller values. Still requires the CANSparkMax and the PIDController created beforehand.
     * @param PIDController Name of the controller to assign values to
     * @param PIDList 1 dimensional array of doubles for the 7 PID values. In order, the values are
     * kP, kI, kD, kFF, KIZone, kMin, and kMax
     */
    public static void setupPIDController(SparkPIDController PIDController, double[] PIDList) {
        m_ControllerChooser.add(PIDController.getName(), PIDController);
        m_PIDList = PIDList;
        PIDController.setP(PIDList[0]);
        PIDController.setI(PIDList[1]);
        PIDController.setD(PIDList[2]);
        PIDController.setFF(PIDList[3]);
        PIDController.setIZone(PIDList[4]);
        PIDController.setOutputRange(PIDList[5], PIDList[6]);
    }

    public static void update() {
        PIDController = m_ControllerChooser.getSelected();
        storeTuningVal(0, SmartDashboard.getNumber("",PIDController.getP());
        storeTuningVal(1, SmartDashboard.getNumber("",PIDController.getI());
        storeTuningVal(2, SmartDashboard.getNumber("",PIDController.getD());
        if (PIDListUpdated){
            SetupPIDController(PIDController, PIDList);
            PIDListUpdated = false;
        }
    }

    public static void storeTuningVal(int index, double tuningVal){
        if (PIDList[index] != tuningVal){
            PIDList[index] = tuningVal;
            PIDListUpdated = true;
        }
    }
}
