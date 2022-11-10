package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;
import frc.robot.util.PIDFFController;
import frc.robot.util.PIDFFGains;

public class SwerveModule extends SubsystemBase {

  enum DrivewheelControlmode {
    PID,
    OPEN_LOOP
  }

  CANSparkMax driver;
  CANSparkMax azimuth;

  SwerveModuleState state;

  OffsetAbsoluteAnalogEncoder azimuthEncoder;

  PIDFFController driveController = new PIDFFController(DriveConstants.kDefaultDrivingGains);
  PIDFFController azimuthController;

  DrivewheelControlmode drivewheelControlmode = DrivewheelControlmode.PID;

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
    driver.setInverted(reverseDriveMotor);

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

  private OffsetAbsoluteAnalogEncoder getAziEncoder() {
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
    double driveOutput = 0;
    if (drivewheelControlmode == DrivewheelControlmode.PID) {
      driveOutput =
          driveController.calculate(getDriveEncoder().getVelocity(), state.speedMetersPerSecond);
    } else if (drivewheelControlmode == DrivewheelControlmode.OPEN_LOOP) {
      driveOutput =
          state.speedMetersPerSecond
              / Constants.DriveConstants.maxSwerveVel
              * RobotController.getBatteryVoltage();
    }
    // driveController.calculate(getDriveEncoder().getVelocity(), state.speedMetersPerSecond);
    final double turnOutput =
        azimuthController.calculate(
            OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
                    Rotation2d.fromDegrees(azimuth.getEncoder().getPosition()))
                .getDegrees(),
            state.angle.getDegrees());

    String moduleId = "[" + driver.getDeviceId() + "|" + azimuth.getDeviceId() + "]";
    String keyPrefix = "Modules/" + moduleId + "/";

    SmartDashboard.putNumber(
        keyPrefix + "azimuth error",
        (OffsetAbsoluteAnalogEncoder.simplifyRotation2d(
                    Rotation2d.fromDegrees(azimuth.getEncoder().getPosition()))
                .getDegrees()
            - state.angle.getDegrees()));
    SmartDashboard.putNumber(
        keyPrefix + "drive velocity error",
        (getDriveEncoder().getVelocity() - state.speedMetersPerSecond));

    driver.setVoltage(driveOutput);
    azimuth.setVoltage(turnOutput);

    SmartDashboard.putNumber(keyPrefix + "real output/driver", driveOutput);
    SmartDashboard.putNumber(keyPrefix + "real output/azimuth", turnOutput);

    SmartDashboard.putNumber(keyPrefix + "neo/azimuth encoder", azimuth.getEncoder().getPosition());
    SmartDashboard.putNumber(keyPrefix + "neo/drive encoder", driver.getEncoder().getPosition());
  }

  @Override
  public void periodic() {
    update();

    String moduleId = "[" + driver.getDeviceId() + "|" + azimuth.getDeviceId() + "]";
    String keyPrefix = "Modules/" + moduleId + "/";

    SmartDashboard.putNumber(
        keyPrefix + "azi encoder/raw volts", azimuthEncoder.getUnadjustedVoltage());
    SmartDashboard.putNumber(
        keyPrefix + "azi encoder/adj volts", azimuthEncoder.getAdjustedVoltage());
    SmartDashboard.putNumber(
        keyPrefix + "azi encoder/adj angle", azimuthEncoder.getAdjustedRotation2d().getDegrees());

    SmartDashboard.putNumber(keyPrefix + "azi encoder/offset", azimuthEncoder.getFilteredOffset());

    SmartDashboard.putNumber(
        keyPrefix + "output/driver",
        driver.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber(
        keyPrefix + "output/azimuth",
        azimuth.getAppliedOutput() * RobotController.getBatteryVoltage());
    SmartDashboard.putNumber(keyPrefix + "target angle", state.angle.getDegrees());
  }
}
