package frc.robot.subsystems.SwerveIO;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {

  @AutoLog
  public static class SwerveInputs {
    public double gyroYawPosition = 0.0;
    public double gyroPitchPosition = 0.0;
    public double gyroRollPosition = 0.0;
    public double gyroCompassHeading = 0.0;
    // public double targetAngle = 0.0;
  }

  public void updateInputs(SwerveInputs inputs);

  public void resetGyro(Rotation2d rotation2d);
}
