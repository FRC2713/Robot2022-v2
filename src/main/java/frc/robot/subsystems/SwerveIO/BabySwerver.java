package frc.robot.subsystems.SwerveIO;

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
import frc.robot.subsystems.SwerveIO.SwerveIO.SwerveInputs;
import frc.robot.subsystems.SwerveIO.module.SwerveModule;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIO;
import frc.robot.util.SwerveHeadingController;

public class BabySwerver extends SubsystemBase {

  SwerveIO io;
  public final SwerveInputs inputs = new SwerveInputs();

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final Pigeon2 gyro = new Pigeon2(RobotMap.pigeonCANId);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(DriveConstants.kinematics, Rotation2d.fromDegrees(gyro.getYaw()));

  public BabySwerver(
      SwerveIO swerveIO,
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight) {
    this.frontLeft = new SwerveModule(frontLeft);
    this.frontRight = new SwerveModule(frontRight);
    this.backLeft = new SwerveModule(backLeft);
    this.backRight = new SwerveModule(backRight);
    io = swerveIO;
    io.updateInputs(inputs);
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

    // frontLeft.setDesiredState(swerveModuleStates[0]);
    // frontRight.setDesiredState(swerveModuleStates[1]);
    // backLeft.setDesiredState(swerveModuleStates[2]);
    // backRight.setDesiredState(swerveModuleStates[3]);
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
    // frontLeft.setDesiredState(swerveModuleStates[0]);
    // frontRight.setDesiredState(swerveModuleStates[1]);
    // backLeft.setDesiredState(swerveModuleStates[2]);
    // backRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
