package frc.robot.util;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.NonNull;
import lombok.experimental.UtilityClass;
import org.littletonrobotics.junction.Logger;

@UtilityClass
public final class RedHawkUtil {
  /**
   * Checks whether the given REVLibError is actually an error, and then logs it to AdvantageScope
   * and SmartDasboard. SmartDashboard variable logged is "RevLibError" and "RevLibError/name"
   * AdvantageScope variable logged is "RevLibError/name"
   *
   * @param status A RevLibError
   * @param name The name of the RevLibError, logged (see description)
   */
  public static void errorHandleSparkMAX(@NonNull REVLibError status, @NonNull String name) {
    String stackTrace = "";
    StackTraceElement[] rawStackTrace = Thread.currentThread().getStackTrace();
    if (status != REVLibError.kOk) {
      for (int i = 0; i < rawStackTrace.length; i++) {
        stackTrace.concat(rawStackTrace[i].toString() + " ");
      }
      Logger.getInstance().recordOutput("RevLibError/" + name, status.name() + " StackTrace: ");
      SmartDashboard.putBoolean("RevLibError/" + name, false);
      SmartDashboard.putBoolean("RevLibError", false);
    }
  }
}
