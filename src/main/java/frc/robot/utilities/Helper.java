package frc.robot.utilities;

import com.revrobotics.SparkPIDController;
public class Helper {
    /**
     * Sets the PID controller values. Still requires the CANSparkMax and the PIDController created beforehand.
     * @param PIDController Name of the controller to assign values to
     * @param PIDList 1 dimensional array of doubles for the 7 PID values. In order, the values are
     * kP, kI, kD, kFF, KIZone, kMin, and kMax
     */
    public static void setupPIDController(SparkPIDController PIDController, double[] PIDList) {
        PIDController.setP(PIDList[0]);
        PIDController.setI(PIDList[1]);
        PIDController.setD(PIDList[2]);
        PIDController.setFF(PIDList[3]);
        PIDController.setIZone(PIDList[4]);
        PIDController.setOutputRange(PIDList[5], PIDList[6]);
    }
}
