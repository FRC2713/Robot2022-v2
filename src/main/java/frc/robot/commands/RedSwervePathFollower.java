package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.RedHolonomicDriveController;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class RedSwervePathFollower extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final SwerveDriveKinematics kinematics;
  private final RedHolonomicDriveController controller;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Field2d field = new Field2d();

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker. This SHOULD NOT contain any commands requiring the same subsystems as this command,
   *     or it will be interrupted
   * @param requirements The subsystems to require.
   */
  public RedSwervePathFollower(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.kinematics = kinematics;
    this.controller = new RedHolonomicDriveController(xController, yController, rotationController);
    this.outputModuleStates = outputModuleStates;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    SmartDashboard.putData("RedSwerve_Field", this.field);
    this.field.getObject("traj").setTrajectory(this.trajectory);

    this.timer.reset();
    this.timer.start();
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);

    Pose2d currentPose = this.poseSupplier.get();
    this.field.setRobotPose(currentPose);

    Logger.getInstance()
        .recordOutput("RedSwerve_xError", currentPose.getX() - desiredState.poseMeters.getX());
    Logger.getInstance()
        .recordOutput("RedSwerve_yError", currentPose.getY() - desiredState.poseMeters.getY());
    Logger.getInstance()
        .recordOutput(
            "RedSwerve_hError",
            currentPose.getRotation().getRadians() - desiredState.holonomicRotation.getRadians());

    ChassisSpeeds targetChassisSpeeds =
        this.controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);
    SwerveModuleState[] targetModuleStates =
        this.kinematics.toSwerveModuleStates(targetChassisSpeeds);

    this.outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
  }
}
