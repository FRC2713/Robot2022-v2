package frc.robot.subsystems.SwerveIO.module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;

public class SwerveModuleIOSparkMAX implements SwerveModuleIO {

  OffsetAbsoluteAnalogEncoder azimuthEncoder;
  CANSparkMax driver;
  CANSparkMax azimuth;

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private OffsetAbsoluteAnalogEncoder getAziEncoder() {
    return azimuthEncoder;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoder().getVelocity(), getAziEncoder().getAdjustedRotation2d());
  }

  public double getVoltageAppliedForCharacterization() {
    return driver.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  public void applyVoltageForCharacterization(double voltage) {
    driver.setVoltage(voltage);
  }

  public SwerveModuleIOSparkMAX(
      int drivePort, int azimPort, int azimuthEncoderPort, double offset) {
    azimuthEncoder = new OffsetAbsoluteAnalogEncoder(azimuthEncoderPort, offset);
    driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.aziEncoderRawVolts = azimuthEncoder.getAdjustedVoltage();
    inputs.aziEncoderAdjVolts = azimuthEncoder.getAdjustedVoltage();
    inputs.aziEncoderAdjAngle = azimuthEncoder.getAdjustedRotation2d().getDegrees();
    inputs.aziOutput = azimuth.getAppliedOutput();

    inputs.driveEncoderPosition = getDriveEncoder().getPosition();
    inputs.driveEncoderVelocity = getDriveEncoder().getVelocity();
    inputs.driveOutput = driver.getAppliedOutput();
  }

  @Override
  public void setAzimuthVoltage(double aziVolts) {
    azimuth.setVoltage(aziVolts);
  }

  @Override
  public void setDriveVoltage(double driveVolts) {
    driver.setVoltage(driveVolts);
  }
}
