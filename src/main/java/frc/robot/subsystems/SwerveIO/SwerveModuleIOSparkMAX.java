package frc.robot.subsystems.SwerveIO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOSparkMAX implements SwerveIO {
  public static class SwerveModuleInputs implements LoggableInputs {
    public double aziEncoderRawVolts = 0.0;
    public double aziEncoderAdjVolts = 0.0;
    public double aziEncoderAdjAngle = 0.0;
    public double outputDrive = 0.0;
    public double outputAzimuth = 0.0;
    public double targetAngle = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("azi encoder/raw volts", aziEncoderRawVolts);
      table.put("azi encoder/adj volts", aziEncoderAdjVolts);
      table.put("azi encoder/adj angle", aziEncoderAdjAngle);
      table.put("output/driver", outputDrive);
      table.put("output/azimuth", outputAzimuth);
      table.put("target angle", targetAngle);
    }

    @Override
    public void fromLog(LogTable table) {
      aziEncoderRawVolts = table.getDouble("azi encoder/raw volts", aziEncoderRawVolts);
      aziEncoderAdjVolts = table.getDouble("azi encoder/adj volts", aziEncoderAdjVolts);
      aziEncoderAdjAngle = table.getDouble("azi encoder/adj angle", aziEncoderAdjAngle);
      outputDrive = table.getDouble("output/driver", outputDrive);
      outputAzimuth = table.getDouble("output/azimuth", outputAzimuth);
      targetAngle = table.getDouble("target angle", targetAngle);
    }
  }

  @Override
  public void updateInputs(SwerveInputs inputs) {
    // TODO Auto-generated method stub

  }
}
