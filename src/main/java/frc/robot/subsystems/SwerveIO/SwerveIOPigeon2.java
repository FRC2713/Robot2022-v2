package frc.robot.subsystems.SwerveIO;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.RobotMap;

public class SwerveIOPigeon2 implements SwerveIO {

  private final Pigeon2 gyro;

  public SwerveIOPigeon2() {
    gyro = new Pigeon2(RobotMap.pigeonCANId);
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.gyroCompassHeading = gyro.getAbsoluteCompassHeading();
    inputs.gyroPitchPosition = gyro.getPitch();
    inputs.gyroRollPosition = gyro.getRoll();
    inputs.gyroYawPosition = gyro.getYaw();
  }

  @Override
  public void resetGyro(Rotation2d rotation2d) {
    gyro.setYaw(rotation2d.getDegrees());
  }
}
