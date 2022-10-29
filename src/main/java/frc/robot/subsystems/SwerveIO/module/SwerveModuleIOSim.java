package frc.robot.subsystems.SwerveIO.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleIOSim implements SwerveModuleIO {

  FlywheelSim azimuthSim = new FlywheelSim(DCMotor.getNEO(1), 7.0 / 150.0, 5);
  double theVolts = 0;

  public SwerveModuleIOSim(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {}

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    azimuthSim.update(0.02);
    inputs.aziEncoderRawVolts = theVolts;
    inputs.aziEncoderAdjVolts = theVolts;
    inputs.aziEncoderAdjAngle = azimuthSim.getAngularVelocityRPM();
    inputs.aziOutput = azimuthSim.getOutput(0);
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    theVolts = volts;
    azimuthSim.setInputVoltage(volts);
  }
}
