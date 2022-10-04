package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {

  CANSparkMax driver;
  CANSparkMax azimuth;

  OffsetAbsoluteAnalogEncoder azimuthEncoder;

  PIDFFController driveController = new PIDFFController(DriveConstants.kDefaultDrivingGains);
  PIDFFController azimuthController = new PIDFFController(DriveConstants.kDefaultAzimuthGains);

  public SwerveModule(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {
    driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);

    getDriveEncoder()
        .setPositionConversionFactor(2 * Math.PI * (Constants.DriveConstants.wheelDiameter / 2));

    azimuthController.enableContinuousInput(-Math.PI, Math.PI);

    azimuthEncoder = new OffsetAbsoluteAnalogEncoder(azimuthEncoderPort, offset);

    azimuth.getEncoder().setPositionConversionFactor(7.0 / 150.0);
    azimuth.getEncoder().setVelocityConversionFactor(7.0 / 150.0);
    azimuth.getEncoder().setPosition(azimuthEncoder.getAdjustedRotation2d().getDegrees() / 360.0);
  }

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

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAziEncoder().getAdjustedRotation2d());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = driveController.calculate(getDriveEncoder().getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = azimuthController.calculate(getAziEncoder().getAdjustedVoltage(),
        state.angle.getRadians());

    driver.setVoltage(driveOutput);
    azimuth.setVoltage(turnOutput);
  }

  @Override
  public void periodic() {
    String moduleId = "[" + driver.getDeviceId() + "/" + azimuth.getDeviceId() + "]";
    String keyPrefix = "Modules/" + moduleId + "/";

    SmartDashboard.putNumber(
        keyPrefix + "azi encoder/raw volts", azimuthEncoder.getUnadjustedVoltage());
    SmartDashboard.putNumber(
        keyPrefix + "azi encoder/adj volts", azimuthEncoder.getAdjustedVoltage());
    SmartDashboard.putNumber(
        keyPrefix + "azi encoder/adj angle", azimuthEncoder.getAdjustedRotation2d().getDegrees());

    SmartDashboard.putNumber(keyPrefix + "output/driver", driver.getAppliedOutput());
    SmartDashboard.putNumber(keyPrefix + "output/azimuth", azimuth.getAppliedOutput());
  }
}
