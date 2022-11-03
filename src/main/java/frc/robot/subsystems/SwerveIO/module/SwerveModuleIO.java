package frc.robot.subsystems.SwerveIO.module;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public class SwerveModuleInputs {
    public double aziAbsoluteEncoderRawVolts = 0.0;
    public double aziAbsoluteEncoderAdjVolts = 0.0;
    public double aziAbsoluteEncoderAdjAngleDeg = 0.0;
    public double aziOutput = 0.0;
    public double aziCurrentDrawAmps = 0.0;
    public double aziEncoderPositionDeg = 0.0;
    public double aziEncoderVelocityDegPerSecond = 0.0;
    public double aziTemp;

    public double driveEncoderPositionMetres = 0.0;
    public double driveEncoderVelocityMetresPerSecond = 0.0;
    public double driveOutput = 0.0;
    public double driveCurrentDrawAmps = 0.0;
    public double driveTemp = 0.0;
  }

  public void updateInputs(SwerveModuleInputs inputs);

  public void setAzimuthVoltage(double aziVolts);

  public void setDriveVoltage(double driveVolts);
}
