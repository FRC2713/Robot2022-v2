// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.BabySwerver;
import frc.robot.util.SwerveHeadingController;
import frc.robot.util.characterization.CharacterizationCommand;
import frc.robot.util.characterization.CharacterizationCommand.FeedForwardCharacterizationData;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  public static final BabySwerver swerveDrive = new BabySwerver();

  public static final XboxController driver = new XboxController(Constants.zero);

  public static final FeedForwardCharacterizationData ffData =
      new FeedForwardCharacterizationData("Module Driving");
  public static final CharacterizationCommand cmd =
      new CharacterizationCommand(
          swerveDrive,
          true,
          ffData,
          (voltage) -> {
            swerveDrive.applyVoltageForCharacterization(voltage);
          },
          () -> swerveDrive.getAverageVelocity());

  @Override
  public void robotInit() {
    swerveDrive.setDefaultCommand(new DefaultDrive());

    new JoystickButton(driver, XboxController.Button.kA.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  swerveDrive.setModuleStates(
                      new SwerveModuleState[] {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                      });
                },
                swerveDrive));
    new JoystickButton(driver, XboxController.Button.kB.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  swerveDrive.setModuleStates(
                      new SwerveModuleState[] {
                        new SwerveModuleState(1, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(1, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(1, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(1, Rotation2d.fromDegrees(90)),
                      });
                },
                swerveDrive));
    new JoystickButton(driver, XboxController.Button.kY.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  // swerveDrive.setModuleStates(
                  //     new SwerveModuleState[] {
                  //       new SwerveModuleState(2, Rotation2d.fromDegrees(180)),
                  //       new SwerveModuleState(2, Rotation2d.fromDegrees(180)),
                  //       new SwerveModuleState(2, Rotation2d.fromDegrees(180)),
                  //       new SwerveModuleState(2, Rotation2d.fromDegrees(180)),
                  //     });
                  SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(0));
                },
                swerveDrive));
    new JoystickButton(driver, XboxController.Button.kX.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  // swerveDrive.setModuleStates(
                  //     new SwerveModuleState[] {
                  //       new SwerveModuleState(3, Rotation2d.fromDegrees(270)),
                  //       new SwerveModuleState(3, Rotation2d.fromDegrees(270)),
                  //       new SwerveModuleState(3, Rotation2d.fromDegrees(270)),
                  //       new SwerveModuleState(3, Rotation2d.fromDegrees(270)),
                  //     });

                  SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(90));
                },
                swerveDrive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    cmd.cancel();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    cmd.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    cmd.cancel();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  public String goFast() {
    return "nyyooooom";
  }
}
