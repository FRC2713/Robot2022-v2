package frc.robot.subsystems.SwerveIO;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.RobotMap;

public class SwerveIOSim implements SwerveIO {

  private final Pigeon2 gyro;

  public SwerveIOSim() {
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
  public void setDesiredState(SwerveModuleState desiredState) {
    // TODO Auto-generated method stub
    
  }
}
