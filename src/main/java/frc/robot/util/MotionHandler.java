package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class MotionHandler {

  public enum MotionMode {
    FULL_DRIVE,
    HEADING_CONTROLLER,
    TRAJECTORY
  }

  public SwerveModuleState[] driveHeadingController() {
    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.kJoystickTurnDeadzone);
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.kJoystickTurnDeadzone);

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                SwerveHeadingController.getInstance().update(),
                Robot.swerveDrive.getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSwerveVel);

    return swerveModuleStates;
  }

  public SwerveModuleState[] driveFullControl() {
    double xSpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftY(), DriveConstants.kJoystickTurnDeadzone);
    double ySpeed =
        MathUtil.applyDeadband(-Robot.driver.getLeftX(), DriveConstants.kJoystickTurnDeadzone);
    double rSpeed =
        MathUtil.applyDeadband(-Robot.driver.getRightX(), DriveConstants.kJoystickTurnDeadzone);

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rSpeed, Robot.swerveDrive.getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSwerveVel);

    return swerveModuleStates;
  }

  public SwerveModuleState[] driveTrajectory() {
    SwerveModuleState[] swerveModuleStates =
        DriveConstants.kinematics.toSwerveModuleStates(TrajectoryController.getInstance().update());

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSwerveVel);

    return swerveModuleStates;
  }
}
