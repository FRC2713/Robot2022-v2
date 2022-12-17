package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.TrajectoryController;

public class testAuto extends SequentialCommandGroup {

  public testAuto() {
    addCommands(
        new InstantCommand(
            () ->
                TrajectoryController.getInstance()
                    .changePath(TrajectoryController.AutoPath.PART_1.getTrajectory())),
        new WaitUntilCommand(() -> TrajectoryController.getInstance().isFinished())
        // new InstantCommand(
        //     () ->
        //         TrajectoryController.getInstance()
        //             .changePath(TrajectoryController.AutoPath.PART_2.getTrajectory()))
        );
  }
}
