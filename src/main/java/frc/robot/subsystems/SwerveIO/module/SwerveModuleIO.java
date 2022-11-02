package frc.robot.subsystems.SwerveIO.module;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public class SwerveModuleInputs {
    public double aziEncoderRawVolts = 0.0;
    public double aziEncoderAdjVolts = 0.0;
    public double aziEncoderAdjAngle = 0.0;
    public double aziOutput = 0.0;

    public double driveEncoderPosition = 0.0;
    public double driveEncoderVelocity = 0.0;
    public double driveOutput = 0.0;
  }

  public void updateInputs(SwerveModuleInputs inputs);

  public void setAzimuthVoltage(double aziVolts);

  public void setDriveVoltage(double driveVolts);
}
