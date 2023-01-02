package frc.robot.subsystems.SwerveIO.module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;

public class SwerveModuleIOSparkMAX implements SwerveModuleIO {

  OffsetAbsoluteAnalogEncoder azimuthEncoder;
  CANSparkMax driver;
  CANSparkMax azimuth;

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private RelativeEncoder getAziEncoder() {
    return azimuth.getEncoder();
  }

  private OffsetAbsoluteAnalogEncoder getAziAbsoluteEncoder() {
    return azimuthEncoder;
  }

  public void applyVoltageForCharacterization(double voltage) {
    driver.setVoltage(voltage);
  }

  public SwerveModuleIOSparkMAX(
      int drivePort, int azimPort, int azimuthEncoderPort, double offset) {

    azimuthEncoder = new OffsetAbsoluteAnalogEncoder(azimuthEncoderPort, offset);
    driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);

    driver.restoreFactoryDefaults();
    azimuth.restoreFactoryDefaults();

    azimuth.setInverted(true);
    driver.setInverted(true);

    driver.setIdleMode(IdleMode.kBrake);
    azimuth.setIdleMode(IdleMode.kBrake);

    getDriveEncoder()
        .setPositionConversionFactor((1.0 / 6.12) * Units.inchesToMeters(4.0) * Math.PI);
    getDriveEncoder()
        .setVelocityConversionFactor((1.0 / 6.12) * Units.inchesToMeters(4.0) * Math.PI / 60);

    getAziEncoder().setPositionConversionFactor(7.0 / 150.0 * 360.0);
    getAziEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360.0);
    getAziEncoder().setPosition(getAziAbsoluteEncoder().getAdjustedRotation2d().getDegrees());
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.aziAbsoluteEncoderRawVolts = azimuthEncoder.getUnadjustedVoltage();
    inputs.aziAbsoluteEncoderAdjVolts = azimuthEncoder.getAdjustedVoltage();
    inputs.aziAbsoluteEncoderAdjAngleDeg = azimuthEncoder.getAdjustedRotation2d().getDegrees();
    inputs.aziOutputVolts = azimuth.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.aziTempCelcius = azimuth.getMotorTemperature();
    inputs.aziCurrentDrawAmps = azimuth.getOutputCurrent();
    inputs.aziEncoderPositionDeg = getAziEncoder().getPosition();
    inputs.aziEncoderVelocityDegPerSecond = getAziEncoder().getVelocity();

    inputs.driveEncoderPositionMetres = getDriveEncoder().getPosition();
    inputs.driveEncoderVelocityMetresPerSecond = getDriveEncoder().getVelocity();
    inputs.driveOutputVolts = driver.getAppliedOutput() * RobotController.getBatteryVoltage();
    inputs.driveCurrentDrawAmps = driver.getOutputCurrent();
    inputs.driveTempCelcius = driver.getMotorTemperature();
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
