// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.SwerveHeadingController;

public class DefaultDrive extends CommandBase {
  /** Creates a new DefaultDrive. */
  public DefaultDrive() {
    addRequirements(Robot.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardReverseInput = Robot.driver.getLeftY();
    double leftRightInput = Robot.driver.getLeftX();
    double rotationalInput = Robot.driver.getRightX();

    double headingControllerDegreesChange = rotationalInput * DriveConstants.driverHeadingControllerRate;
    Rotation2d newHeadingSetpoint = SwerveHeadingController.getInstance().getSetpoint()
        .plus(Rotation2d.fromDegrees(headingControllerDegreesChange));

    SwerveHeadingController.getInstance().setSetpoint(newHeadingSetpoint);

    Robot.swerveDrive.drive(leftRightInput, forwardReverseInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveHeadingController.getInstance().setSetpoint(Robot.swerveDrive.getPose().getRotation());
    Robot.swerveDrive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
