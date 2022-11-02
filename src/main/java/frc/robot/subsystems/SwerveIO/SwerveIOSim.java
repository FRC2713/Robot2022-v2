package frc.robot.subsystems.SwerveIO;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveIOSim implements SwerveIO {

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.gyroCompassHeading = 0;
    inputs.gyroPitchPosition = 0;
    inputs.gyroRollPosition = 0;
    inputs.gyroYawPosition = 0;
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // TODO Auto-generated method stub

  }
}
