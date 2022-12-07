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

public class TrajectoryController {
  private static TrajectoryController instance;
  Timer timer = new Timer();
  PathPlannerTrajectory traj =
      PathPlanner.loadPath("taxitaxi", PathPlanner.getConstraintsFromPath("taxitaxi"));
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

  public void loadPath(PathPlannerTrajectory newTrajectory) {
    traj = newTrajectory;
    timer.reset();
  }

  public ChassisSpeeds update() {
    if (timer.get() == 0) {
      timer.start();
    }
    PathPlannerState targetState = (PathPlannerState) traj.sample(timer.get());

    if (traj.getTotalTimeSeconds() < timer.get()) {
      return controller.calculate(Robot.swerveDrive.getPose(), targetState);
    } else return new ChassisSpeeds();
  }
}
