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
import frc.robot.subsystems.SwerveIO.BabySwerver;
import frc.robot.subsystems.SwerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.SwerveIO.SwerveIOSim;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.util.characterization.CharacterizationCommand;
import frc.robot.util.characterization.CharacterizationCommand.FeedForwardCharacterizationData;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.io.LogSocketServer;

public class Robot extends LoggedRobot {
  public static final BabySwerver swerveDrive =
      new BabySwerver(
          Robot.isReal() ? new SwerveIOPigeon2() : new SwerveIOSim(),
          Robot.isReal()
              ? new SwerveModuleIOSparkMAX(
                  Constants.RobotMap.frontLeftDrive,
                  Constants.RobotMap.frontLeftAzi,
                  Constants.RobotMap.frontLeftAzimuthEncoder,
                  Constants.RobotMap.frontLeftOffset)
              : new SwerveModuleIOSim(
                  Constants.RobotMap.frontLeftDrive,
                  Constants.RobotMap.frontLeftAzi,
                  Constants.RobotMap.frontLeftAzimuthEncoder,
                  Constants.RobotMap.frontLeftOffset),
          Robot.isReal()
              ? new SwerveModuleIOSparkMAX(
                  Constants.RobotMap.frontRightDrive,
                  Constants.RobotMap.frontRightAzi,
                  Constants.RobotMap.frontRightAzimuthEncoder,
                  Constants.RobotMap.frontRightOffset)
              : new SwerveModuleIOSim(
                  Constants.RobotMap.frontRightDrive,
                  Constants.RobotMap.frontRightAzi,
                  Constants.RobotMap.frontRightAzimuthEncoder,
                  Constants.RobotMap.frontRightOffset),
          Robot.isReal()
              ? new SwerveModuleIOSparkMAX(
                  Constants.RobotMap.backLeftDrive,
                  Constants.RobotMap.backLeftAzi,
                  Constants.RobotMap.backLeftAzimuthEncoder,
                  Constants.RobotMap.backLeftOffset)
              : new SwerveModuleIOSim(
                  Constants.RobotMap.backLeftDrive,
                  Constants.RobotMap.backLeftAzi,
                  Constants.RobotMap.backLeftAzimuthEncoder,
                  Constants.RobotMap.backLeftOffset),
          Robot.isReal()
              ? new SwerveModuleIOSparkMAX(
                  Constants.RobotMap.backRightDrive,
                  Constants.RobotMap.backRightAzi,
                  Constants.RobotMap.backRightAzimuthEncoder,
                  Constants.RobotMap.backRightOffset)
              : new SwerveModuleIOSim(
                  Constants.RobotMap.backRightDrive,
                  Constants.RobotMap.backRightAzi,
                  Constants.RobotMap.backRightAzimuthEncoder,
                  Constants.RobotMap.backRightOffset));

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
          () -> swerveDrive.getAverageVoltageAppliedForCharacterization());

  @Override
  public void robotInit() {
    swerveDrive.setDefaultCommand(new DefaultDrive());
    Logger.getInstance().addDataReceiver(new LogSocketServer(5800));
    Logger.getInstance().start();

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
    new JoystickButton(driver, XboxController.Button.kY.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  swerveDrive.setModuleStates(
                      new SwerveModuleState[] {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
                      });
                },
                swerveDrive));
    new JoystickButton(driver, XboxController.Button.kX.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  swerveDrive.setModuleStates(
                      new SwerveModuleState[] {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
                      });
                },
                swerveDrive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

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
