package frc.robot.util;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.experimental.UtilityClass;
import org.littletonrobotics.junction.Logger;

@UtilityClass
public final class RedHawkUtil {
  public static void errorHandleSparkMAX(REVLibError status, String name) {
    if (status != REVLibError.kOk) {
      Logger.getInstance().recordOutput("RevLibError/" + name, status.name());
      SmartDashboard.putBoolean("RevLibError/" + name, false);
      SmartDashboard.putBoolean("RevLibError", false);
    }
  }
}
