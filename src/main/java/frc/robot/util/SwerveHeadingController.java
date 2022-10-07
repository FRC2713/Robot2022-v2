package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class SwerveHeadingController {
  private static SwerveHeadingController instance;
  private Rotation2d setpoint;
  private PIDFFController controller;

  private SwerveHeadingController() {
    controller = new PIDFFController(DriveConstants.kHeadingControllerGains);
    controller.setTolerance(DriveConstants.kHeadingControllerGains.tolerance.get());
    controller.enableContinuousInput(-180, 180);

    setpoint = Robot.swerveDrive.getPose().getRotation();
  }

  public static SwerveHeadingController getInstance() {
    if (instance == null) {
      instance = new SwerveHeadingController();
    }

    return instance;
  }

  public void setSetpoint(Rotation2d setpoint) {
    this.setpoint = setpoint;
  }

  public Rotation2d getSetpoint() {
    return setpoint;
  }

  public double update() {
    if (Constants.tuningMode) {
      setSetpoint(
          Rotation2d.fromDegrees(
              SmartDashboard.getNumber(
                  "Heading Controller/setpoint degrees", setpoint.getDegrees())));
    } else {
      SmartDashboard.putNumber("Heading Controller/setpoint degrees", setpoint.getDegrees());
    }

    Rotation2d currentHeading = Robot.swerveDrive.getPose().getRotation();
    double output = controller.calculate(currentHeading.getDegrees(), setpoint.getDegrees());
    SmartDashboard.putNumber("Heading Controller/update", output);
    return output;
  }
}
