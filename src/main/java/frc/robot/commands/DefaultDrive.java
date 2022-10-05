// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.BabySwerver;
import frc.robot.util.SwerveHeadingController;

public class DefaultDrive extends CommandBase {
  XboxController driver;
  BabySwerver swerver;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(XboxController driver, BabySwerver swerver) {
    this.driver = driver;
    this.swerver = swerver;
    addRequirements(swerver);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardReverseInput = driver.getLeftY();
    double leftRightInput = driver.getLeftX();
    double rotationalInput = driver.getRightX();

    double headingControllerDegreesChange =
        rotationalInput * DriveConstants.headingControllerDriverChangeRate;
    Rotation2d newHeadingSetpoint =
        SwerveHeadingController.getInstance()
            .getSetpoint()
            .plus(Rotation2d.fromDegrees(headingControllerDegreesChange));

    SwerveHeadingController.getInstance().setSetpoint(newHeadingSetpoint);

    swerver.drive(leftRightInput, forwardReverseInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveHeadingController.getInstance().setSetpoint(swerver.getPose().getRotation());
    swerver.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
