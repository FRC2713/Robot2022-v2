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

  /**
   * Updates swerve positional values given gyro configurations??????????????????????????????????
   *
   * @param inputs Gyro configurations??????????????????????????
   */
  public void updateInputs(SwerveInputs inputs);

  /**
   * Sets the gyro to the given rotation2d
   *
   * @param rotation2d Gyro rotation
   */
  public void resetGyro(Rotation2d rotation2d);
}
