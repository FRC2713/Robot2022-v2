// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.subsystems.BabySwerver;
import frc.robot.util.characterization.CharacterizationCommand;
import frc.robot.util.characterization.CharacterizationCommand.FeedForwardCharacterizationData;
import java.util.HashMap;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {

  public Command autoCommand;

  public static final BabySwerver swerveDrive = new BabySwerver();

  public static final XboxController driver = new XboxController(Constants.zero);

  private SendableChooser<Command> autoSelect = new SendableChooser<>();

  public static final FeedForwardCharacterizationData ffData =
      new FeedForwardCharacterizationData("Module Driving");

  public static PathPlannerTrajectory taxi;

  public static final CharacterizationCommand characterization =
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
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                      });
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
    if (autoCommand != null) {

      autoCommand.cancel();
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

    HashMap<String, Command> eventMap = new HashMap<>();

    taxi = PathPlanner.loadPath("taxitaxi", PathPlanner.getConstraintsFromPath("taxitaxi"));

    autoCommand =
        new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                  swerveDrive.resetGyro(taxi.getInitialHolonomicPose().getRotation());
                  swerveDrive.resetOdometry(taxi.getInitialHolonomicPose());
                }),
            new PPSwerveControllerCommand(
                taxi,
                () -> swerveDrive.getPose(),
                Constants.DriveConstants.kinematics,
                new PIDController(0.9, 0, 0),
                new PIDController(0.9, 0, 0),
                new PIDController(0.5, 0, 0),
                (states) -> {
                  swerveDrive.setModuleStates(states);
                },
                eventMap,
                swerveDrive));

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
