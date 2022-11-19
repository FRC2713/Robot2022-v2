package frc.robot.subsystems.SwerveIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveIO.module.SwerveModule;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIO;
import org.littletonrobotics.junction.Logger;

public class BabySwerver extends SubsystemBase {

  SwerveIO io;
  public final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveOdometry odometry;
  private Pose2d simOdometryPose;

  public BabySwerver(
      SwerveIO swerveIO,
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight) {
    this.frontLeft = new SwerveModule(frontLeft, "Front left");
    this.frontRight = new SwerveModule(frontRight, "Front right");
    this.backLeft = new SwerveModule(backLeft, "Back left");
    this.backRight = new SwerveModule(backRight, "Back right");
    io = swerveIO;
    io.updateInputs(inputs);

    odometry = new SwerveDriveOdometry(
        DriveConstants.kinematics, Rotation2d.fromDegrees(inputs.gyroYawPosition));
    simOdometryPose = odometry.getPoseMeters();
  }

  public Pose2d getPose() {
    if (Robot.isReal()) {
      return odometry.getPoseMeters();
    } else {
      return simOdometryPose;
    }
  }

  public void drive(double xSpeed, double ySpeed, double rSpeed) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * Constants.DriveConstants.maxSwerveVel,
            ySpeed * Constants.DriveConstants.maxSwerveVel,
            rSpeed * Constants.DriveConstants.maxRotationalSpeedDegPerSec,
            getPose().getRotation()));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.DriveConstants.maxSwerveVel);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
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
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());

    if (Robot.isSimulation()) {
      SwerveModuleState[] measuredStates = new SwerveModuleState[] { frontLeft.getState(),
          frontRight.getState(),
          backLeft.getState(),
          backRight.getState() };
          ChassisSpeeds speeds = Constants.DriveConstants.kinematics.toChassisSpeeds(measuredStates);
          simOdometryPose = simOdometryPose.exp(new Twist2d(
            speeds.vxMetersPerSecond * .02,
            speeds.vyMetersPerSecond * .02,
            speeds.omegaRadiansPerSecond * .02));
    }
  }

  public void setModuleStates(SwerveModuleState swerveModuleStates[]) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateOdometry();
    Logger.getInstance().processInputs("Swerve/Chassis", inputs);
    Logger.getInstance()
        .recordOutput(
            "Swerve/Odometry",
            new double[] {
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getDegrees()
            });
  }
}
