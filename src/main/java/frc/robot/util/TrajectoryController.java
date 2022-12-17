package frc.robot.util;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import java.util.HashMap;
import lombok.NonNull;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {
  public enum AutoPath {
    PART_1("autopart1"),
    PART_2("autopart2");

    private PathPlannerTrajectory ppTrajectory;

    private AutoPath(String filename) {
      this.ppTrajectory =
          PathPlanner.loadPath(filename, PathPlanner.getConstraintsFromPath(filename));
    }

    public PathPlannerTrajectory getTrajectory() {
      return ppTrajectory;
    }
  }

  private static TrajectoryController instance;
  Timer timer = new Timer();
  PathPlannerTrajectory traj = AutoPath.PART_1.getTrajectory();
  HashMap<String, Command> eventMap = new HashMap<>();
  PPHolonomicDriveController controller =
      new PPHolonomicDriveController(
          new PIDController(0.9, 0, 0), new PIDController(0.9, 0, 0), new PIDController(1.0, 0, 0));

  private TrajectoryController() {}

  public static TrajectoryController getInstance() {
    if (instance == null) {
      instance = new TrajectoryController();
    }
    return instance;
  }

  public void changePath(@NonNull PathPlannerTrajectory newTrajectory) {
    traj = newTrajectory;
    timer.reset();
  }

  public boolean isFinished() {
    return timer.get() >= traj.getTotalTimeSeconds();
  }

  public ChassisSpeeds update() {
    if (timer.get() == 0) {
      timer.start();
    }

    PathPlannerState targetState = (PathPlannerState) traj.sample(timer.get());

    Logger.getInstance()
        .recordOutput(
            "Trajectory/Target Pose",
            new double[] {
              targetState.poseMeters.getX(),
              targetState.poseMeters.getY(),
              targetState.holonomicRotation.getDegrees()
            });
    Logger.getInstance().recordOutput("Trajectory/timer", timer.get());
    if (!isFinished()) {
      return controller.calculate(Robot.swerveDrive.getPose(), targetState);
    } else return new ChassisSpeeds();
  }
}
