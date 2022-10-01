package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;
import frc.robot.util.SwerveHeadingController;
import frc.robot.util.SwerveModule;

public class BabySwerver extends SubsystemBase {
  private final Translation2d frontLeftLocation = new Translation2d(0.5, 0.5);
  private final Translation2d frontRightLocation = new Translation2d(0.5, -0.5);
  private final Translation2d backLeftLocation = new Translation2d(-0.5, 0.5);
  private final Translation2d backRightLocation = new Translation2d(-0.5, -0.5);

  private final SwerveModule frontLeft =
      new SwerveModule(
          Constants.RobotMap.frontLeftDrive,
          Constants.RobotMap.frontLeftAzi,
          Constants.RobotMap.frontLeftAzimuthEncoder,
          Constants.RobotMap.frontLeftOffset);
  private final SwerveModule frontRight =
      new SwerveModule(
          Constants.RobotMap.frontRightDrive,
          Constants.RobotMap.frontRightAzi,
          Constants.RobotMap.frontRightAzimuthEncoder,
          Constants.RobotMap.frontRightOffset);
  private final SwerveModule backLeft =
      new SwerveModule(
          Constants.RobotMap.backLeftDrive,
          Constants.RobotMap.backLeftAzi,
          Constants.RobotMap.backLeftAzimuthEncoder,
          Constants.RobotMap.backLeftOffset);
  private final SwerveModule backRight =
      new SwerveModule(
          Constants.RobotMap.backRightDrive,
          Constants.RobotMap.backRightAzi,
          Constants.RobotMap.backRightAzimuthEncoder,
          Constants.RobotMap.backRightOffset);

  private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonCANId);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getYaw()));

  public BabySwerver() {
    gyro.zeroGyroBiasNow();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void drive(double xSpeed, double ySpeed) {
    SwerveModuleState[] swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                SwerveHeadingController.getInstance().update(),
                getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSwerveVel);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void updateOdometry() {
    odometry.update(
        Rotation2d.fromDegrees(gyro.getYaw()),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }
}
