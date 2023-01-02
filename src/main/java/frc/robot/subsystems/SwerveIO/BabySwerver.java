package frc.robot.subsystems.SwerveIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
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

  /**
   * Creates a new BabySwerver (swerve drive) object.
   *
   * @param swerveIO The IO layer of the swerve drive. Change this to change which gyro you're using
   *     (SwerveModuleIOPigeon2 vs SwerveModuleIOSim)
   * @param frontLeft The IO layer for the front left swerve module. Change this to change which
   *     motor controller you're using (SwerveModuleIOSim vs SwerveModuleIOSparkMAX)
   * @param frontRight The IO layer for the front right swerve module.
   * @param backLeft The IO layer for the back left swerve module.
   * @param backRight The IO layer for the back left swerve module.
   */
  public BabySwerver(
      SwerveIO swerveIO,
      SwerveModuleIO frontLeft,
      SwerveModuleIO frontRight,
      SwerveModuleIO backLeft,
      SwerveModuleIO backRight) {
    this.frontLeft = new SwerveModule(frontLeft, "Front Left");
    this.frontRight = new SwerveModule(frontRight, "Front Right");
    this.backLeft = new SwerveModule(backLeft, "Back Left");
    this.backRight = new SwerveModule(backRight, "Back Right");
    io = swerveIO;
    io.updateInputs(inputs);

    odometry =
        new SwerveDriveOdometry(
            DriveConstants.kinematics, Rotation2d.fromDegrees(inputs.gyroYawPosition));
    simOdometryPose = odometry.getPoseMeters();
  }

  /**
   * Sets the gyro to the given rotation.
   *
   * @param rotation The rotation to reset the gyro to.
   */
  public void resetGyro(Rotation2d rotation) {
    io.resetGyro(rotation);
  }

  /**
   * Resets the SwerveDriveOdometry to the given pose.
   *
   * @param pose The desired pose.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, pose.getRotation());
    simOdometryPose = pose;
  }

  /**
   * Returns the current pose of the robot.
   *
   * @return The position of the robot on the field.
   */
  public Pose2d getPose() {
    if (Robot.isReal()) {
      return odometry.getPoseMeters();
    } else {
      return simOdometryPose;
    }
  }

  /**
   * Sets the desired states of the swerve modules.
   *
   * @param swerveModuleStates The array of desired swerveModuleStates. Ensure they are ordered the
   *     same way in this array as they are instantiated into SwerveDriveKinematics.
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Returns the average velocity of the swerve modules.
   *
   * @return The average velocity at which all the swerve modules are moving.
   */
  public double getAverageVelocity() {
    return (frontLeft.getState().speedMetersPerSecond
            + frontRight.getState().speedMetersPerSecond
            + backLeft.getState().speedMetersPerSecond
            + backRight.getState().speedMetersPerSecond)
        / 4;
  }

  // Only used for characterization
  public void applyVoltageForCharacterization(double volts) {
    frontLeft.applyVoltageForCharacterization(volts);
    frontRight.applyVoltageForCharacterization(volts);
    backLeft.applyVoltageForCharacterization(volts);
    backRight.applyVoltageForCharacterization(volts);
  }

  /**
   * Updates the odometry of the robot using the swerve module states and the gyro reading. Should
   * be run in periodic() or during every code loop to maintain accuracy.
   */
  public void updateOdometry() {
    odometry.update(
        Rotation2d.fromDegrees(inputs.gyroYawPosition),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());

    if (Robot.isSimulation()) {
      SwerveModuleState[] measuredStates =
          new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
          };
      ChassisSpeeds speeds = Constants.DriveConstants.kinematics.toChassisSpeeds(measuredStates);
      simOdometryPose =
          simOdometryPose.exp(
              new Twist2d(
                  speeds.vxMetersPerSecond * .02,
                  speeds.vyMetersPerSecond * .02,
                  speeds.omegaRadiansPerSecond * .02));
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    updateOdometry();

    switch (Robot.motionMode) {
      case FULL_DRIVE:
        setModuleStates(Robot.motionHandler.driveFullControl());
        break;
      case HEADING_CONTROLLER:
        setModuleStates(Robot.motionHandler.driveHeadingController());
        break;
      case LOCKDOWN:
        setModuleStates(Robot.motionHandler.lockdown());
        break;
      case TRAJECTORY:
        setModuleStates(Robot.motionHandler.driveTrajectory());
        break;
      default:
        break;
    }

    Logger.getInstance().processInputs("Swerve/Chassis", inputs);
    Logger.getInstance()
        .recordOutput(
            "Swerve/Odometry",
            new double[] {
              getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees()
            });
    Logger.getInstance().recordOutput("Swerve/MotionMode", Robot.motionMode.name());
  }
}
