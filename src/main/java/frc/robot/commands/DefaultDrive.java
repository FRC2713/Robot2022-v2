// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.util.MotionHandler;
import frc.robot.util.SwerveHeadingController;
import frc.robot.util.MotionHandler.MotionMode;

public class DefaultDrive extends CommandBase {
  /** Creates a new DefaultDrive. */
  public DefaultDrive() {
    addRequirements(Robot.swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.motionMode == MotionMode.FULL_DRIVE) {
    Robot.swerveDrive.drive(Robot.motionHandler.driveFullControl());
    }
    else if(Robot.motionMode == MotionMode.HEADING_CONTROLLER) {
      Robot.swerveDrive.drive(Robot.motionHandler.driveHeadingController());
    }
    else if(Robot.motionMode == MotionMode.TRAJECTORY) {
      Robot.swerveDrive.drive(Robot.motionHandler.driveTrajectory());
    }
    else{
      Robot.swerveDrive.drive(Robot.motionHandler.lockdown());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveHeadingController.getInstance().setSetpoint(Robot.swerveDrive.getPose().getRotation());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
