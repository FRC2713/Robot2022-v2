// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.testAuto;
import frc.robot.subsystems.SwerveIO.BabySwerver;
import frc.robot.subsystems.SwerveIO.SwerveIOPigeon2;
import frc.robot.subsystems.SwerveIO.SwerveIOSim;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIOSparkMAX;
import frc.robot.util.MotionHandler;
import frc.robot.util.MotionHandler.MotionMode;
import frc.robot.util.TrajectoryController;
import frc.robot.util.characterization.CharacterizationCommand.FeedForwardCharacterizationData;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.io.LogSocketServer;

public class Robot extends LoggedRobot {

  public Command autoCommand;

  public static MotionMode motionMode = MotionMode.FULL_DRIVE;

  public static final MotionHandler motionHandler = new MotionHandler();

  /**
   * Sets up swerve modules for all four wheels, both for the simulator and the physical SparkMAX.
   * Checks if a real swerve module or a simulated module. Takes in the IO being used (which is
   * configured to be changeable), and the four swerve modules that this wonderful robot uses.
   */
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

  public static PathPlannerTrajectory part1, part2;

  /**
   * Robot initialization. Doesn't require extra variables, which is nice. Currently used mostly to
   * store keyinputs, Including drive mode changes and gyro resets for the swerve to standard angle
   * measures (in this case 90 and 270 degrees)
   */
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

    new JoystickButton(driver, XboxController.Button.kB.value)
        .whenPressed(
            new InstantCommand(
                () -> {
                  motionMode = MotionMode.HEADING_CONTROLLER;
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

  /**
   * Robot periodic, it runs periodically. All it does now is run the command scheduler. Worth
   * keeping around.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * For when the robot is disabled. Cancels autocommand and sets the motion mode to LOCKDOWN This
   * turns off movement and also makes the robot generally harder to move. Like some defense
   * configuartion in your favorite video game
   */
  @Override
  public void disabledInit() {
    if (autoCommand != null) {

      autoCommand.cancel();
    }

    Robot.motionMode = MotionMode.LOCKDOWN;
  }

  /** Periodic stuff that occurs while disabled. Empty currently. */
  @Override
  public void disabledPeriodic() {}

  /**
   * Initialization for autonomous programming. Path planner loads the "taxitaxi" program. Also,
   * sets the motion mode to trajectory
   */
  @Override
  public void autonomousInit() {

    autoSelect.setDefaultOption(
        "autopart1",
        new InstantCommand(() -> TrajectoryController.getInstance().changePath(part1)));

    autoCommand = new testAuto();

    swerveDrive.resetOdometry(
        TrajectoryController.AutoPath.PART_1.getTrajectory().getInitialHolonomicPose());

    if (autoCommand != null) {
      motionMode = MotionMode.TRAJECTORY;
      autoCommand.schedule();
    }
  }

  /** Autonomous stuff that is called regularly. Isn't used right now! */
  @Override
  public void autonomousPeriodic() {}

  /**
   * Initialization for the Teleop mode. Cancels autocommands and sets motion to lockdown, then
   * reverts it to FULLDRIVE This is so our robot can be ready to go crazy on the field (see you at
   * world 2023 I'm putting a lot of hope into that)
   */
  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      motionMode = MotionMode.LOCKDOWN;
      autoCommand.cancel();
    }
    Robot.motionMode = MotionMode.FULL_DRIVE;
  }

  /** Periodic updates during the Teleop (manual control). Isn't used! */
  @Override
  public void teleopPeriodic() {}

  /**
   * Testing initialization code goes here. All it does right now is cancel all commands in the
   * command scheduler. Neat.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Periodic updates during testing modes. Not used */
  @Override
  public void testPeriodic() {}

  /** Contains vital code. Do not delete or code may break. */
  public String goFast() {
    return "nyyooooom";
  }
}
