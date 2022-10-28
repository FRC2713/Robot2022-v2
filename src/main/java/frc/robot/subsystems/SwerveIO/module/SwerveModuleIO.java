package frc.robot.subsystems.SwerveIO.module;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {

  @AutoLog
  public static class SwerveInputs {
    public double aziEncoderRawVolts = 0.0;
    public double aziEncoderAdjVolts = 0.0;
    public double aziEncoderAdjAngle = 0.0;
    public double outputDriver = 0.0;
    public double aziOutput = 0.0;
  }

  public void updateInputs(SwerveInputs inputs);
  
  public void setAzimuthVoltage(double volts);
}
