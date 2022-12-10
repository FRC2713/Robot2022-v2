package frc.robot.subsystems.SwerveIO.module.modules;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveIO.module.SwerveModule;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIO;
import frc.robot.subsystems.SwerveIO.module.SwerveModules;
import frc.robot.util.PIDFFController;

public class FrontRightModule extends SwerveModule {
  public FrontRightModule(SwerveModuleIO swerveModuleIO) {
    super(
        swerveModuleIO,
        SwerveModules.FRONT_RIGHT,
        new PIDFFController(DriveConstants.Gains.FrontRight.kDefaultDrivingGains),
        new PIDFFController(DriveConstants.Gains.FrontRight.kDefaultAzimuthGains));
  }
}
