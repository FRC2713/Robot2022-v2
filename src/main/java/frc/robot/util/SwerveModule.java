package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

public class SwerveModule {

  CANSparkMax driver;
  CANSparkMax azimuth;

  OffsetAbsoluteAnalogEncoder azimuthEncoder;

  private final PIDController drivePID = new PIDController(1, 0, 0);
  private final PIDController aziPID = new PIDController(1, 0, 0);

  private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward aziFF = new SimpleMotorFeedforward(0, 0);

  public SwerveModule(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {
    driver = new CANSparkMax(drivePort, MotorType.kBrushless);
    azimuth = new CANSparkMax(azimPort, MotorType.kBrushless);

    getDriveEncoder()
        .setPositionConversionFactor(2 * Math.PI * (Constants.DriveConstants.wheelDiameter / 2));

    aziPID.enableContinuousInput(-Math.PI, Math.PI);

    azimuthEncoder = new OffsetAbsoluteAnalogEncoder(azimuthEncoderPort, offset);
  }

  private RelativeEncoder getDriveEncoder() {
    return driver.getEncoder();
  }

  private OffsetAbsoluteAnalogEncoder getAziEncoder() {
    return azimuthEncoder;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoder().getVelocity(), getAziEncoder().getRotation2d());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAziEncoder().getRotation2d());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = drivePID.calculate(getDriveEncoder().getVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = driveFF.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = aziPID.calculate(getAziEncoder().getAdjustedVoltage(), state.angle.getRadians());

    final double turnFeedforward = aziFF.calculate(aziPID.getSetpoint());

    driver.setVoltage(driveOutput + driveFeedforward);
    azimuth.setVoltage(turnOutput + turnFeedforward);
  }
}
