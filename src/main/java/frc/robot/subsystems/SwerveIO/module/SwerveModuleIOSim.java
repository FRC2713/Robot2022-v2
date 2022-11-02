package frc.robot.subsystems.SwerveIO.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleIOSim implements SwerveModuleIO {

  FlywheelSim azimuthSim = new FlywheelSim(DCMotor.getNEO(0), 7.0 / 150.0, 5);
  FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 7.0 / 150.0, 5);
  double theAziVolts = 0;
  double theDriveVolts = 0;

  public SwerveModuleIOSim(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {}

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    azimuthSim.update(0.02);
    inputs.aziEncoderRawVolts = theAziVolts;
    inputs.aziEncoderAdjVolts = theAziVolts;
    inputs.aziEncoderAdjAngle = azimuthSim.getAngularVelocityRPM() / 0.02;
    inputs.aziOutput = azimuthSim.getOutput(0);

    inputs.driveEncoderVelocity = driveSim.getAngularVelocityRPM();
    inputs.driveEncoderPosition = driveSim.getAngularVelocityRPM() / 0.02;
    inputs.driveOutput = driveSim.getOutput(1);
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    theAziVolts = aziVolts;
    azimuthSim.setInputVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    theDriveVolts = driveVolts;
    driveSim.setInputVoltage(driveVolts);
  }
}
