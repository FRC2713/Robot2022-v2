package frc.robot.util;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class RedHawkUtil {
  private RedHawkUtil() {
    throw new AssertionError();
  }

  public static void ErrorHandleSparkMAX(REVLibError status) {
    if (status != REVLibError.kOk) {
      Logger.getInstance().recordOutput("RevLibError", status.name());
      SmartDashboard.putBoolean("RevLibError", false);
    }
  }
}
