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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.BabySwerver;
import frc.robot.util.characterization.CharacterizationCommand;
import frc.robot.util.characterization.CharacterizationCommand.FeedForwardCharacterizationData;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
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

  public static SequentialCommandGroup taxi =
      new SequentialCommandGroup(
          new RunCommand(
              () -> {
                    swerveDrive.setModuleStates(
                        new SwerveModuleState[] {
                          new SwerveModuleState(3, Rotation2d.fromDegrees(180)),
                          new SwerveModuleState(3, Rotation2d.fromDegrees(180)),
                          new SwerveModuleState(3, Rotation2d.fromDegrees(180)),
                          new SwerveModuleState(3, Rotation2d.fromDegrees(180))
                        });
                  }, swerveDrive)
                  .withTimeout(5),
              stop);

    public SequentialCommandGroup setTaxi(double degrees) {
      return new SequentialCommandGroup(
        new RunCommand(
          () -> {
                swerveDrive.setModuleStates(
                    new SwerveModuleState[] {
                      new SwerveModuleState(3, Rotation2d.fromDegrees(degrees)),
                      new SwerveModuleState(3, Rotation2d.fromDegrees(degrees)),
                      new SwerveModuleState(3, Rotation2d.fromDegrees(degrees)),
                      new SwerveModuleState(3, Rotation2d.fromDegrees(degrees))}
                      );
              }, swerveDrive),
          stop);
    }

  @Override
  public void robotInit() {
    swerveDrive.setDefaultCommand(new DefaultDrive());

    autoSelect.addOption(
        "Left Fender",
        new RunCommand(
            () -> {
              swerveDrive.resetGyro(Rotation2d.fromDegrees(130));
              taxi = setTaxi(180);
            },
            swerveDrive));

    autoSelect.addOption(
        "Left Tape",
        new RunCommand(
            () -> {
              swerveDrive.resetGyro(Rotation2d.fromDegrees(180));
            },
            swerveDrive));

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
    taxi.cancel();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    taxi.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    taxi.cancel();
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
