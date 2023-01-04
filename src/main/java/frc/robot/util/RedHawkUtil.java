package frc.robot.util;

import com.revrobotics.REVLibError;
import java.util.ArrayList;
import java.util.List;
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
    if (status != REVLibError.kOk) {
      StackTraceElement[] rawStackTrace = Thread.currentThread().getStackTrace();
      ErrHandler.getInstance()
          .addError(
              status.name()
                  + " StackTrace: "
                  + rawStackTrace[2].getFileName()
                  + ":"
                  + rawStackTrace[2].getLineNumber());
    }
  }

  public static class ErrHandler {
    private static ErrHandler INSTANCE;

    /**
     * Gets the instance of the ErrHandler singleton
     *
     * @return The one instance of ErrHandler
     */
    public static ErrHandler getInstance() {
      if (INSTANCE == null) {
        INSTANCE = new ErrHandler();
      }
      return INSTANCE;
    }

    private ErrHandler() {}

    private List<String> errors = new ArrayList<>();

    /**
     * Adds an error to the list of RevLib errors. Must be called on the singleton (see getInstance)
     */
    public void addError(@NonNull String error) {
      this.errors.add(error);
    }

    public void log() {
      Logger.getInstance().recordOutput("RevLibErrors", String.join(" \\\\ ", errors));
    }
  }
}
