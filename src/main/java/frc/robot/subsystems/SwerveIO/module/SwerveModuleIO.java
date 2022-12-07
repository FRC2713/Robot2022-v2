package frc.robot.subsystems.SwerveIO.module;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public class SwerveModuleInputs {
    public double aziAbsoluteEncoderRawVolts = 0.0;
    public double aziAbsoluteEncoderAdjVolts = 0.0;
    public double aziAbsoluteEncoderAdjAngleDeg = 0.0;
    public double aziOutputVolts = 0.0;
    public double aziCurrentDrawAmps = 0.0;
    public double aziEncoderPositionDeg = 0.0;
    public double aziEncoderSimplifiedPositionDeg = 0.0;
    public double aziEncoderVelocityDegPerSecond = 0.0;
    public double aziTempCelcius;

    public double driveEncoderPositionMetres = 0.0;
    public double driveEncoderVelocityMetresPerSecond = 0.0;
    public double driveOutputVolts = 0.0;
    public double driveCurrentDrawAmps = 0.0;
    public double driveTempCelcius = 0.0;
  }

  public void updateInputs(SwerveModuleInputs inputs);

  public void setAzimuthVoltage(double aziVolts);

  public void setDriveVoltage(double driveVolts);
}
