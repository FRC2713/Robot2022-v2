package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BabySwerver;

public class BabySwerving extends CommandBase {
  BabySwerver babySwerver;

  double xTarget;
  double yTarget;
  double rTarget;

  double xChange;
  double yChange;
  double rChange;

  double xSpeed;
  double ySpeed;
  double rSpeed;

  public BabySwerving(double xTarget, double yTarget, BabySwerver babySwerver) {
    this.xTarget = xTarget;
    this.yTarget = yTarget;
    this.babySwerver = babySwerver;

    addRequirements(babySwerver);
  }

  @Override
  public void initialize() {
    babySwerver.updateOdometry();
    xChange = xTarget - babySwerver.getPose().getX();
    yChange = yTarget - babySwerver.getPose().getY();
  }

  @Override
  public void execute() {
    // babySwerver.drive(xChange, yChange);
  }

  @Override
  public boolean isFinished() {
    if (babySwerver.getPose().getY() == yTarget && babySwerver.getPose().getX() == xTarget) {
      return true;
    } else return false;
  }

  @Override
  public void end(boolean interrupted) {
    // babySwerver.drive(0, 0);
  }
}
