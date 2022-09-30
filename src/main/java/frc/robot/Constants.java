// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// liam sais hi :)
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean tuningMode = false;
  public static final int zero = 0; // in case you need a zero :)

  public static final class RobotMap {
    public static final int pigeonCANId = 20;

    // MOTORS

    public static final int frontLeftDrive = 1;
    public static final int frontLeftAzi = 2;
    public static final int frontLeftAzimuthEncoder = 0;

    public static final double frontLeftOffset = 0;
    // placeholder
    public static final int backLeftDrive = 3;
    public static final int backLeftAzi = 4;
    public static final int backLeftAzimuthEncoder = 1;

    public static final double backLeftOffset = 0;
    // placeholder
    public static final int frontRightDrive = 5;
    public static final int frontRightAzi = 6;
    public static final int frontRightAzimuthEncoder = 2;

    public static final double frontRightOffset = 0;
    // placeholder
    public static final int backRightDrive = 7;
    public static final int backRightAzi = 8;
    public static final int backRightAzimuthEncoder = 3;

    public static final double backRightOffset = 0;
    // placeholder

    public static final int flywheelLeftPort = 5;
    public static final int flywheelRightPort = 10;
    public static final int flywheelTopLeft = 13;
    public static final int flywheelTopRight = 55;

    public static final int intakeMotorRollers = 7;
    public static final int intakeMotorFourBar = 6;
    public static final int intakeMotorFourBar2 = 8;

    public static final int lowerSnek = 50;
    public static final int upperSnek = 9;

    public static final int climberMotorRight = 11;
    public static final int climberMotorLeft = 12;

    // DIO

    public static final int snekLowerSwitch = 3;
    public static final int snekUpperSwitch = 1;

    // PWM

    public static final int stripPort = 0;
  }

  public static final class DriveConstants {
    public static final double kJoystickTurnDeadzone = 0.04;
    public static final double wheelDiameter = 5;
    public static final double gearRatio = 60.0 / 11.0 * 28.0 / 20; // 60.0 / 10.0;
    public static final double distPerPulse = (1.0 / gearRatio) * Units.inchesToMeters(wheelDiameter) * Math.PI;

    public static final double maxSwerveVel = 3;
    public static final double maxSwerveAzi = Math.PI;

    public static final int currentLimit = 65;

    private static final double bumperlessRobotLength = Units.inchesToMeters(26);
    private static final double bumperlessRobotWidth = Units.inchesToMeters(24);
    private static final double bumperThickness = Units.inchesToMeters(3);

    public static final double fullRobotWidth = bumperlessRobotWidth + bumperThickness * 2;
    public static final double fullRobotLength = bumperlessRobotLength + bumperThickness * 2;

    public static final double headingControllerkP = 0.1;
    public static final double headingControllerkI = 0.0;
    public static final double headingControllerkD = 0.0;
    public static final double headingControllerTolerance = 0.0;

    public static final double driverHeadingControllerRate = 0.1;
  }

  public static final class AutoConstants {
    // FF and FB gains; NEED TO BE DETERMINED ON THE FULLY BUILT ROBOT, WILL CHANGE
    // WITH WEIGHT
    public static final double ksVolts = 0.15161; // 0.15166; // 0.20541;
    public static final double kvVoltSecondsPerMeter = 2.511; // 2.5108; // 2.4361;
    public static final double kaVoltSecondsSquaredPerMeter = 0.34892; // 0.34944; // 0.25946;

    public static final double kPDriveVel = 5.7664; // 2.9805; // 3.95;

    // more kinematics stuff
    public static final double trackWidth = Units.inchesToMeters(22);
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);

    public static final double maxCentripetalAcceleration = 1.5;

    // Ramsete constants; generally the same on all robots
    public static final double RamseteZeta = 0.7;
    public static final double RamseteB = 2;

    // Max speeds
    public static final double maxSpeed = Units.feetToMeters(15);
    public static final double maxAccel = Units.feetToMeters(10);
    public static final double maxVoltageApplied = 10;

    // swerve constants
    public static final double swerveMaxVel = Units.feetToMeters(3);
    public static final double swerveMacAccel = Units.feetToMeters(0.5);

    public static final double aziMaxVel = Units.feetToMeters(0.5);
    public static final double aziMaxAccel = Units.feetToMeters(0.1);

    // five ball
    public static final double waitForHumanPlayerDuration = 0.3;
    public static final double crawlTowardsHumanPlayerVolts = 0.6;
  }
}
