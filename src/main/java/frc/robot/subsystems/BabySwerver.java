package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.Logger;

public class BabySwerver extends SubsystemBase {
  private final SwerveModule frontLeft =
      new SwerveModule(
          Constants.RobotMap.frontLeftDrive,
          Constants.RobotMap.frontLeftAzi,
          Constants.RobotMap.frontLeftAzimuthEncoder,
          Constants.RobotMap.frontLeftOffset,
          Constants.DriveConstants.kFrontLeftAzimuthGains,
          false);
  private final SwerveModule frontRight =
      new SwerveModule(
          Constants.RobotMap.frontRightDrive,
          Constants.RobotMap.frontRightAzi,
          Constants.RobotMap.frontRightAzimuthEncoder,
          Constants.RobotMap.frontRightOffset,
          Constants.DriveConstants.kFrontRightAzimuthGains,
          false);
  private final SwerveModule backLeft =
      new SwerveModule(
          Constants.RobotMap.backLeftDrive,
          Constants.RobotMap.backLeftAzi,
          Constants.RobotMap.backLeftAzimuthEncoder,
          Constants.RobotMap.backLeftOffset,
          Constants.DriveConstants.kBackLeftAzimuthGains,
          false);
  private final SwerveModule backRight =
      new SwerveModule(
          Constants.RobotMap.backRightDrive,
          Constants.RobotMap.backRightAzi,
          Constants.RobotMap.backRightAzimuthEncoder,
          Constants.RobotMap.backRightOffset,
          Constants.DriveConstants.kBackRightAzimuthGains,
          false);

  private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonCANId);

  private final SwerveDriveOdometry odometry;

  public BabySwerver() {
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
    odometry =
        new SwerveDriveOdometry(DriveConstants.kinematics, Rotation2d.fromDegrees(gyro.getYaw()));
    // gyro.setYaw(90);
    // gyro.config
  }

  public void resetGyro(Rotation2d rotation) {
    gyro.setYaw(rotation.getDegrees());
  }

  public void resetOdometry(Pose2d pose) {
    // gyro.setYaw(pose.getRotation().getDegrees());
    odometry.resetPosition(pose, pose.getRotation());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void drive(double xSpeed, double ySpeed, double rSpeed) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed * DriveConstants.maxSwerveVel,
                ySpeed * DriveConstants.maxSwerveVel,
                rSpeed * Math.PI * 2,
                getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSwerveVel);

    setModuleStates(swerveModuleStates);
  }

  public double getAverageVelocity() {
    return (frontLeft.getState().speedMetersPerSecond
            + frontRight.getState().speedMetersPerSecond
            + backLeft.getState().speedMetersPerSecond
            + backRight.getState().speedMetersPerSecond)
        / 4;
  }

  public void applyVoltageForCharacterization(double volts) {
    frontLeft.applyVoltageForCharacterization(volts);
    frontRight.applyVoltageForCharacterization(volts);
    backLeft.applyVoltageForCharacterization(volts);
    backRight.applyVoltageForCharacterization(volts);
  }

  public void updateOdometry() {
    odometry.update(
        Rotation2d.fromDegrees(gyro.getYaw()),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public void setModuleStates(SwerveModuleState swerveModuleStates[]) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    updateOdometry();

    Logger.getInstance().recordOutput("Gyro/Yaw", gyro.getYaw());
    Logger.getInstance().recordOutput("Gyro/Pitch", gyro.getPitch());
    Logger.getInstance().recordOutput("Gyro/Roll", gyro.getRoll());
    Logger.getInstance().recordOutput("Gyro/Compass Heading", gyro.getAbsoluteCompassHeading());

    Logger.getInstance().recordOutput("Odometry/X", odometry.getPoseMeters().getX());
    Logger.getInstance().recordOutput("Odometry/Y", odometry.getPoseMeters().getY());
    Logger.getInstance()
        .recordOutput("Odometry/H", odometry.getPoseMeters().getRotation().getDegrees());
  }
}
