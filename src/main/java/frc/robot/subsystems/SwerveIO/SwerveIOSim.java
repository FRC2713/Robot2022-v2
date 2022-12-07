package frc.robot.subsystems.SwerveIO;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveIOSim implements SwerveIO {

  /** Initializes gyro inputs to default values aka 0? */
  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.gyroCompassHeading = 0;
    inputs.gyroPitchPosition = 0;
    inputs.gyroRollPosition = 0;
    inputs.gyroYawPosition = 0;
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {}
}
