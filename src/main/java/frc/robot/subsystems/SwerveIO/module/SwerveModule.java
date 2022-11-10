package frc.robot.subsystems.SwerveIO.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.PIDFFController;

public class SwerveModule extends SubsystemBase {

  SwerveModuleIO io;
  // SwerveModuleInputs inputs = new SwerveModuleInputs();
  public final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

  // CANSparkMax driver;
  // CANSparkMax azimuth;

  SwerveModuleState state;

  // OffsetAbsoluteAnalogEncoder azimuthEncoder;

  PIDFFController driveController = new PIDFFController(DriveConstants.kDefaultDrivingGains);
  PIDFFController azimuthController = new PIDFFController(DriveConstants.kDefaultAzimuthGains);

  public SwerveModule(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {
    io = new SwerveModuleIOSparkMAX(drivePort, azimPort, azimuthEncoderPort, offset);
    io.updateInputs(inputs);
    // driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    // azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);

    // driver.setIdleMode(IdleMode.kBrake);
    // azimuth.setIdleMode(IdleMode.kBrake);

    // getDriveEncoder()
    //     .setPositionConversionFactor(2 * Math.PI * (Constants.DriveConstants.wheelDiameter / 2));

    // azimuthController.enableContinuousInput(-Math.PI, Math.PI);

    // azimuthEncoder = new OffsetAbsoluteAnalogEncoder(azimuthEncoderPort, offset);

    state = new SwerveModuleState(0, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
    azimuthController.enableContinuousInput(-Math.PI, Math.PI);
    // state = new SwerveModuleState(0, inputs.
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveEncoderVelocityMetresPerSecond,
        Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }

  public double getVoltageAppliedForCharacterization() {
    return inputs.driveOutputVolts;
  }

  public void applyVoltageForCharacterization(double voltage) {
    io.setDriveVoltage(voltage);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    state = desiredState;
  }

  public void update() {
    final double driveOutput =
        driveController.calculate(
            inputs.driveEncoderVelocityMetresPerSecond, state.speedMetersPerSecond);
    final double turnOutput =
        azimuthController.calculate(inputs.aziAbsoluteEncoderAdjAngleDeg, state.angle.getDegrees());

    applyVoltageForCharacterization(driveOutput);
    io.setAzimuthVoltage(turnOutput);
  }

  @Override
  public void periodic() {
    update();

    io.updateInputs(inputs);
  }
}
