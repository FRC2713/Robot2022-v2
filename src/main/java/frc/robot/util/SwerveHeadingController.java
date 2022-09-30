package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class SwerveHeadingController {
    private static SwerveHeadingController instance;
    private Rotation2d setpoint;
    private PIDController controller;

    private SwerveHeadingController() {
        controller = new PIDController(DriveConstants.headingControllerkP, DriveConstants.headingControllerkI,
                DriveConstants.headingControllerkD);
        controller.setTolerance(DriveConstants.headingControllerTolerance);
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

    public double update() {
        Rotation2d currentHeading = Robot.swerveDrive.getPose().getRotation();
        double output = controller.calculate(currentHeading.getDegrees(), setpoint.getDegrees());
        return output;
    }
}
