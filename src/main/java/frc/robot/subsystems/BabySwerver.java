package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.util.SwerveHeadingController;

public class BabySwerver extends SubsystemBase {
  private final SwerveModule frontLeft =
      new SwerveModule(
          Constants.RobotMap.frontLeftDrive,
          Constants.RobotMap.frontLeftAzi,
          Constants.RobotMap.frontLeftAzimuthEncoder,
          Constants.RobotMap.frontLeftOffset,
          Constants.DriveConstants.kFrontLeftAzimuthGains);
  private final SwerveModule frontRight =
      new SwerveModule(
          Constants.RobotMap.frontRightDrive,
          Constants.RobotMap.frontRightAzi,
          Constants.RobotMap.frontRightAzimuthEncoder,
          Constants.RobotMap.frontRightOffset,
          Constants.DriveConstants.kFrontRightAzimuthGains);
  private final SwerveModule backLeft =
      new SwerveModule(
          Constants.RobotMap.backLeftDrive,
          Constants.RobotMap.backLeftAzi,
          Constants.RobotMap.backLeftAzimuthEncoder,
          Constants.RobotMap.backLeftOffset,
          Constants.DriveConstants.kBackLeftAzimuthGains);
  private final SwerveModule backRight =
      new SwerveModule(
          Constants.RobotMap.backRightDrive,
          Constants.RobotMap.backRightAzi,
          Constants.RobotMap.backRightAzimuthEncoder,
          Constants.RobotMap.backRightOffset,
          Constants.DriveConstants.kBackRightAzimuthGains);

  private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonCANId);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(DriveConstants.kinematics, Rotation2d.fromDegrees(gyro.getYaw()));

  public BabySwerver() {
    gyro.zeroGyroBiasNow();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void drive(double xSpeed, double ySpeed) {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                SwerveHeadingController.getInstance().update(),
                getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSwerveVel);

    setModuleStates(swerveModuleStates);
  }

  public double getAverageVoltageAppliedForCharacterization() {
    return (frontLeft.getVoltageAppliedForCharacterization()
            + frontRight.getVoltageAppliedForCharacterization()
            + backLeft.getVoltageAppliedForCharacterization()
            + backRight.getVoltageAppliedForCharacterization())
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
    frontLeft.setDesiredState(
        SwerveModuleState.optimize(swerveModuleStates[0], frontLeft.getState().angle));
    frontRight.setDesiredState(
        SwerveModuleState.optimize(swerveModuleStates[1], frontRight.getState().angle));
    backLeft.setDesiredState(
        SwerveModuleState.optimize(swerveModuleStates[2], backLeft.getState().angle));
    backRight.setDesiredState(
        SwerveModuleState.optimize(swerveModuleStates[3], backRight.getState().angle));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro/Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Gyro/Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Gyro/Roll", gyro.getRoll());
    SmartDashboard.putNumber("Gyro/Compass Heading", gyro.getAbsoluteCompassHeading());
  }
}
