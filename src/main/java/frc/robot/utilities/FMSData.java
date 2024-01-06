package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FMSConstants;

public class FMSData {
    
  private int allianceColor;
  private Alliance alliance;

  // findColor - sets the global variable allianceColor based on the color of the
  // alliance coming from the fmsdata
  public int getAllianceColor() {
    try {
      // robot is ENABLED
      if (RobotController.isSysActive()) {
        // connected to FMS
        allianceColor = FMSConstants.ALLIANCE_INITIALIZED; // 0
        alliance = DriverStation.getAlliance();

        String gameData = DriverStation.getGameSpecificMessage();
        SmartDashboard.putString("gameData", gameData);

        if (alliance == Alliance.Blue) {
          allianceColor = FMSConstants.ALLIANCE_BLUE; // -1
        } else if (alliance == Alliance.Red) {
          allianceColor = FMSConstants.ALLIANCE_RED; // 1
        } else {
          allianceColor = FMSConstants.ALLIANCE_INITIALIZED; // 0
        }
      } else { // robot is NOT ENABLED
        allianceColor = FMSConstants.ALLIANCE_NOT_ENABLED; // 20
      }

    } catch (Exception e) {
      allianceColor = FMSConstants.ALLIANCE_EXCEPTION; // 11
    }

    // example on updating the dashboard using the smart dashboard implementation
    SmartDashboard.putNumber("Alliance", allianceColor);

    // exmaple on updating the dashboard using the Shuffleboard custom widget and
    // the network table entry
    // allianceColorEntry.setDouble(allianceColor);
    return allianceColor;
  }
}
