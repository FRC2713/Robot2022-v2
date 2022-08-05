package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SwerveModule;

public class BabySwerver extends SubsystemBase {
  private final Translation2d frontLeftLocation = new Translation2d(0.5, 0.5);
  private final Translation2d frontRightLocation = new Translation2d(0.5, -0.5);
  private final Translation2d backLeftLocation = new Translation2d(-0.5, 0.5);
  private final Translation2d backRightLocation = new Translation2d(-0.5, -0.5);

  private final SwerveModule frontLeft =
      new SwerveModule(Constants.RobotMap.frontLeftDrive, Constants.RobotMap.frontLeftAzi);
  private final SwerveModule frontRight =
      new SwerveModule(Constants.RobotMap.frontRightDrive, Constants.RobotMap.frontRightAzi);
  private final SwerveModule backLeft =
      new SwerveModule(Constants.RobotMap.backLeftDrive, Constants.RobotMap.backLeftAzi);
  private final SwerveModule backRight =
      new SwerveModule(Constants.RobotMap.backRightDrive, Constants.RobotMap.backRightAzi);

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

  public BabySwerver() {
    gyro.reset();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void drive(double xSpeed, double ySpeed, double angle) {
    SwerveModuleState[] swerveModuleStates =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angle, gyro.getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSwerveVel);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }
}
