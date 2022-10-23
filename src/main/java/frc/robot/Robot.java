// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.BabySwerver;
import frc.robot.util.characterization.CharacterizationCommand;
import frc.robot.util.characterization.CharacterizationCommand.FeedForwardCharacterizationData;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {

  public Command autoCommand;

  public static final BabySwerver swerveDrive = new BabySwerver();

  public static final XboxController driver = new XboxController(Constants.zero);

  private SendableChooser<Command> autoSelect = new SendableChooser<>();

  public static final FeedForwardCharacterizationData ffData =
      new FeedForwardCharacterizationData("Module Driving");

  public static final CharacterizationCommand characterization =
      new CharacterizationCommand(
          swerveDrive,
          true,
          ffData,
          (voltage) -> {
            swerveDrive.applyVoltageForCharacterization(voltage);
          },
          () -> swerveDrive.getAverageVelocity());

  public static final Command taxitaxi =
      new RunCommand(
          () -> {
            swerveDrive.setModuleStates(
                new SwerveModuleState[] {
                  new SwerveModuleState(3, Rotation2d.fromDegrees(90)),
                  new SwerveModuleState(3, Rotation2d.fromDegrees(90)),
                  new SwerveModuleState(3, Rotation2d.fromDegrees(90)),
                  new SwerveModuleState(3, Rotation2d.fromDegrees(90))
                });
          },
          swerveDrive);

  public static final Command stop =
      new RunCommand(
          () -> {
            swerveDrive.setModuleStates(
                new SwerveModuleState[] {
                  new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                  new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                  new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                  new SwerveModuleState(0, Rotation2d.fromDegrees(90))
                });
          },
          swerveDrive);

  // public SequentialCommandGroup setTaxi(double degrees) {
  //   return new SequentialCommandGroup(
  //       new RunCommand(
  //               () -> {
  //                 swerveDrive.setModuleStates(
  //                     new SwerveModuleState[] {
  //                       new SwerveModuleState(3, Rotation2d.fromDegrees(degrees)),
  //                       new SwerveModuleState(3, Rotation2d.fromDegrees(degrees)),
  //                       new SwerveModuleState(3, Rotation2d.fromDegrees(degrees)),
  //                       new SwerveModuleState(3, Rotation2d.fromDegrees(degrees))
  //                     });
  //               },
  //               swerveDrive)
  //           .withTimeout(5),
  //       stop);
  // }

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
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                      });
                  // SwerveHeadingController.getInstance().setSetpoint(Rotation2d.fromDegrees(135));
                },
                swerveDrive));

    // autoSelect.addOption(
    //     "Left Fender",
    //     new SequentialCommandGroup(
    //         new InstantCommand(
    //             () -> {
    //               swerveDrive.resetGyro(Rotation2d.fromDegrees(130));
    //             },
    //             swerveDrive),
    //         setTaxi(130)));

    // autoSelect.addOption(
    //     "Left Tape",
    //     new SequentialCommandGroup(
    //         new RunCommand(
    //             () -> {
    //               swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
    //             },
    //             swerveDrive),
    //         setTaxi(180)));

    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  swerveDrive.resetGyro(Rotation2d.fromDegrees(90));
                }));

    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  swerveDrive.resetGyro(Rotation2d.fromDegrees(270));
                }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    if (autoCommand != null) {

      autoCommand.cancel();
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if (autoSelect.getSelected() != null) {
      autoCommand = autoSelect.getSelected();
    }
    if (autoCommand != null) {

      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {

      autoCommand.cancel();
    }
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
