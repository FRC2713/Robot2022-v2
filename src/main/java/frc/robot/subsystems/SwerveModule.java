package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;
import frc.robot.util.PIDFFController;
import frc.robot.util.PIDFFGains;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

  CANSparkMax driver;
  CANSparkMax azimuth;

  SwerveModuleState state;

  OffsetAbsoluteAnalogEncoder azimuthEncoder;

  PIDFFController driveController = new PIDFFController(DriveConstants.kDefaultDrivingGains);
  PIDFFController azimuthController;

  // PIDController azimuthController = new
  // PIDController(DriveConstants.kDefaultAzimuthGains.kP.get(),
  // DriveConstants.kDefaultAzimuthGains.kI.get(),
  // DriveConstants.kDefaultAzimuthGains.kD.get());

  public SwerveModule(
      int drivePort,
      int azimPort,
      int azimuthEncoderPort,
      double offset,
      PIDFFGains azimuthGains,
      boolean reverseDriveMotor) {
    driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);

    driver.restoreFactoryDefaults();
    azimuth.restoreFactoryDefaults();

    azimuth.setInverted(true);
    driver.setInverted(true);

    azimuthController = new PIDFFController(azimuthGains);

    driver.setIdleMode(IdleMode.kBrake);
    azimuth.setIdleMode(IdleMode.kBrake);

    driver.setSmartCurrentLimit(50);
    azimuth.setSmartCurrentLimit(20);

    getDriveEncoder().setPositionConversionFactor(DriveConstants.distPerPulse);
    getDriveEncoder().setVelocityConversionFactor(DriveConstants.distPerPulse / 60);

    azimuthController.enableContinuousInput(-180, 180);

    azimuthEncoder = new OffsetAbsoluteAnalogEncoder(azimuthEncoderPort, offset);

    state = new SwerveModuleState(0, azimuthEncoder.getAdjustedRotation2d());

    azimuth.getEncoder().setPositionConversionFactor(7.0 / 150.0 * 360);
    azimuth.getEncoder().setVelocityConversionFactor(7.0 / 150.0 * 360);
    azimuth.getEncoder().setPosition(azimuthEncoder.getAdjustedRotation2d().getDegrees());

    driver.burnFlash();
    azimuth.burnFlash();
  }

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private OffsetAbsoluteAnalogEncoder getAnalogEncoder() {
    return azimuthEncoder;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoder().getVelocity(),
        OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
            Rotation2d.fromDegrees(azimuth.getEncoder().getPosition())));
  }

  public double getVoltageAppliedForCharacterization() {
    return driver.getAppliedOutput() * RobotController.getBatteryVoltage();
  }

  public void applyVoltageForCharacterization(double voltage) {
    driver.setVoltage(voltage);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    state =
        SwerveModuleState.optimize(
            desiredState,
            OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
                Rotation2d.fromDegrees(azimuth.getEncoder().getPosition())));
  }

  public void update() {
    final double driveOutput =
        driveController.calculate(getDriveEncoder().getVelocity(), state.speedMetersPerSecond);
    final double turnOutput =
        azimuthController.calculate(
            OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
                    Rotation2d.fromDegrees(azimuth.getEncoder().getPosition()))
                .getDegrees(),
            state.angle.getDegrees());

    String moduleId = "[" + driver.getDeviceId() + "|" + azimuth.getDeviceId() + "]";
    String keyPrefix = "Modules/" + moduleId + "/";

    Logger.getInstance()
        .recordOutput(
            keyPrefix + "azimuth error",
            (OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
                        Rotation2d.fromDegrees(azimuth.getEncoder().getPosition()))
                    .getDegrees()
                - state.angle.getDegrees()));
    Logger.getInstance()
        .recordOutput(
            keyPrefix + "drive velocity error",
            (getDriveEncoder().getVelocity() - state.speedMetersPerSecond));

    driver.setVoltage(driveOutput);
    azimuth.setVoltage(turnOutput);

    Logger.getInstance().recordOutput(keyPrefix + "real output/driver", driveOutput);
    Logger.getInstance().recordOutput(keyPrefix + "real output/azimuth", turnOutput);

    Logger.getInstance()
        .recordOutput(keyPrefix + "neo/azimuth encoder", azimuth.getEncoder().getPosition());
    Logger.getInstance()
        .recordOutput(keyPrefix + "neo/drive encoder", driver.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    update();

    String moduleId = "[" + driver.getDeviceId() + "|" + azimuth.getDeviceId() + "]";
    String keyPrefix = "Modules/" + moduleId + "/";

    Logger.getInstance()
        .recordOutput(keyPrefix + "azi encoder/raw volts", azimuthEncoder.getUnadjustedVoltage());
    Logger.getInstance()
        .recordOutput(keyPrefix + "azi encoder/adj volts", azimuthEncoder.getAdjustedVoltage());
    Logger.getInstance()
        .recordOutput(
            keyPrefix + "azi encoder/adj angle",
            azimuthEncoder.getAdjustedRotation2d().getDegrees());

    Logger.getInstance()
        .recordOutput(keyPrefix + "azi encoder/offset", azimuthEncoder.getFilteredOffset());

    Logger.getInstance()
        .recordOutput(
            keyPrefix + "output/driver",
            driver.getAppliedOutput() * RobotController.getBatteryVoltage());
    Logger.getInstance()
        .recordOutput(
            keyPrefix + "output/azimuth",
            azimuth.getAppliedOutput() * RobotController.getBatteryVoltage());
    Logger.getInstance().recordOutput(keyPrefix + "target angle", state.angle.getDegrees());
  }
}
