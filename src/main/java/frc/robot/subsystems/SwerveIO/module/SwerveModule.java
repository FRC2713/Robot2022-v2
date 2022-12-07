package frc.robot.subsystems.SwerveIO.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.PIDFFController;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

  SwerveModuleIO io;
  String name;
  // SwerveModuleInputs inputs = new SwerveModuleInputs();
  public final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

  // CANSparkMax driver;
  // CANSparkMax azimuth;

  SwerveModuleState state;

  // OffsetAbsoluteAnalogEncoder azimuthEncoder;

  PIDFFController driveController = new PIDFFController(DriveConstants.kDefaultDrivingGains);
  PIDFFController azimuthController = new PIDFFController(DriveConstants.kDefaultAzimuthGains);

  public SwerveModule(SwerveModuleIO swerveModuleIO, String name) {
    io = swerveModuleIO;
    this.name = name;
    io.updateInputs(inputs);

    state = new SwerveModuleState(0, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
    azimuthController.enableContinuousInput(-180, 180);
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
    state =
        SwerveModuleState.optimize(
            desiredState, Rotation2d.fromDegrees(inputs.aziEncoderPositionDeg));
  }

  public void update() {
    final double driveOutput =
        driveController.calculate(
            inputs.driveEncoderVelocityMetresPerSecond, state.speedMetersPerSecond);
    final double turnOutput =
        azimuthController.calculate(inputs.aziEncoderPositionDeg, state.angle.getDegrees());

    applyVoltageForCharacterization(driveOutput);
    io.setAzimuthVoltage(turnOutput);
  }

  @Override
  public void periodic() {
    update();

    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Swerve/" + name, inputs);
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + name + "/Azimuth Error",
            state.angle.getDegrees() - inputs.aziEncoderPositionDeg);
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + name + "/Drive Error",
            state.speedMetersPerSecond - inputs.driveEncoderVelocityMetresPerSecond);

    Logger.getInstance()
        .recordOutput(
            "Swerve/" + name + "/Azimuth Encoder Delta",
            inputs.aziEncoderPositionDeg - inputs.aziAbsoluteEncoderAdjAngleDeg);
  }
}
