package frc.robot.subsystems.SwerveIO;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveIO {

  @AutoLog
  public static class SwerveInputs {
    public double gyroYawPosition = 0.0;
    public double gyroPitchPosition = 0.0;
    public double gyroRollPosition = 0.0;
    public double gyroCompassHeading = 0.0;
    public double targetAngle = 0.0;
  }

  public void updateInputs(SwerveInputs inputs);

  public void setDesiredState(SwerveModuleState desiredState)

}
