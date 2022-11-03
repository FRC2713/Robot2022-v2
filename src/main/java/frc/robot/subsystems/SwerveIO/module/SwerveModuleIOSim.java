package frc.robot.subsystems.SwerveIO.module;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleIOSim implements SwerveModuleIO {

  FlywheelSim azimuthSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 5);
  FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.12, 5);

  double theAziVolts = 0;
  double theDriveVolts = 0;

  public SwerveModuleIOSim(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {}

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    azimuthSim.update(0.02);
    driveSim.update(0.02);

    inputs.aziAbsoluteEncoderRawVolts = 0;
    inputs.aziAbsoluteEncoderAdjVolts = 0;
    inputs.aziAbsoluteEncoderAdjAngle = 0;
    inputs.aziOutput = azimuthSim.getOutput(0);
    inputs.aziTemp = 0.0;
    inputs.aziCurrentDraw = azimuthSim.getCurrentDrawAmps();
    inputs.aziEncoderPosition += Units.radiansToDegrees(azimuthSim.getAngularVelocityRadPerSec()) * 0.02;
    inputs.aziEncoderVelocity = Units.radiansToDegrees(azimuthSim.getAngularVelocityRadPerSec());

    inputs.driveEncoderVelocity = driveSim.getAngularVelocityRPM();
    inputs.driveEncoderPosition = driveSim.getAngularVelocityRPM() / 0.02;
    inputs.driveOutput = driveSim.getOutput(0);
    inputs.driveCurrentDraw = driveSim.getCurrentDrawAmps();
    inputs.driveTemp = 0.0;
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
