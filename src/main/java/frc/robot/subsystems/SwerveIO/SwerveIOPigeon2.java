package frc.robot.subsystems.SwerveIO;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.RobotMap;

public class SwerveIOPigeon2 implements SwerveIO {

  private final Pigeon2 gyro;

  // SwerveModuleState state;

  public SwerveIOPigeon2() {
    gyro = new Pigeon2(RobotMap.pigeonCANId);
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
    // state = new SwerveModuleState(0, azimuthEncoder.getAdjustedRotation2d());
  }

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.gyroCompassHeading = gyro.getAbsoluteCompassHeading();
    inputs.gyroPitchPosition = gyro.getPitch();
    inputs.gyroRollPosition = gyro.getRoll();
    inputs.gyroYawPosition = gyro.getYaw();
    // inputs.targetAngle = state.angle.getDegrees();
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // TODO Auto-generated method stub

  }
}
