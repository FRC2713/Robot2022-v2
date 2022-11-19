// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveIO.BabySwerver;
import frc.robot.subsystems.SwerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.SwerveIO.SwerveIOSim;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.util.MotionHandler;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.TrajectoryController;
import frc.robot.util.characterization.CharacterizationCommand.FeedForwardCharacterizationData;
import java.util.HashMap;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.io.LogSocketServer;

public class Robot extends LoggedRobot {

  public Command autoCommand;

  public static MotionMode motionMode = MotionMode.FULL_DRIVE;

  public static final MotionHandler motionHandler = new MotionHandler();

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

  private SendableChooser<Command> autoSelect = new SendableChooser<>();

  public static final FeedForwardCharacterizationData ffData =
      new FeedForwardCharacterizationData("Module Driving");

  public static PathPlannerTrajectory taxi;

  @Override
  public void robotInit() {
    // setUseTiming(isReal());
    // LoggedNetworkTables.getInstance().addTable("/SmartDashboard");
    Logger.getInstance().addDataReceiver(new LogSocketServer(5800));
    Logger.getInstance().start();

    new JoystickButton(driver, XboxController.Button.kY.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.LOCKDOWN;
                }));

    new JoystickButton(driver, XboxController.Button.kA.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.FULL_DRIVE;
                }));

    new JoystickButton(driver, XboxController.Button.kStart.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  if (motionMode == MotionMode.FULL_DRIVE) {
                    motionMode = MotionMode.HEADING_CONTROLLER;
                  }
                  if (motionMode == MotionMode.HEADING_CONTROLLER) {
                    motionMode = MotionMode.FULL_DRIVE;
                  }
                }));

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

    Robot.motionMode = MotionMode.LOCKDOWN;
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
                new PIDController(1.0, 0, 0),
                (states) -> {
                  swerveDrive.setModuleStates(states);
                },
                eventMap,
                swerveDrive));

    autoSelect.addOption(
        "taxitaxi", new InstantCommand(() -> TrajectoryController.getInstance().loadPath(taxi)));

    autoCommand = autoSelect.getSelected();

    if (autoCommand != null) {
      motionMode = MotionMode.TRAJECTORY;
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      motionMode = MotionMode.LOCKDOWN;
      autoCommand.cancel();
    }
    Robot.motionMode = MotionMode.FULL_DRIVE;
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
