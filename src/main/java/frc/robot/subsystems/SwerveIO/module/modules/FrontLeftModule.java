package frc.robot.subsystems.SwerveIO.module.modules;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveIO.module.SwerveModule;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIO;
import frc.robot.subsystems.SwerveIO.module.SwerveModules;
import frc.robot.util.PIDFFController;

public class FrontLeftModule extends SwerveModule {
  public FrontLeftModule(SwerveModuleIO swerveModuleIO) {
    super(
        swerveModuleIO,
        SwerveModules.FRONT_LEFT,
        new PIDFFController(DriveConstants.Gains.FrontLeft.kDefaultDrivingGains),
        new PIDFFController(DriveConstants.Gains.FrontLeft.kDefaultAzimuthGains));
  }
}
