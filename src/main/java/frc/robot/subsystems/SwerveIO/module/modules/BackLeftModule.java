package frc.robot.subsystems.SwerveIO.module.modules;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveIO.module.SwerveModule;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIO;
import frc.robot.subsystems.SwerveIO.module.SwerveModules;
import frc.robot.util.PIDFFController;

public class BackLeftModule extends SwerveModule {
  public BackLeftModule(SwerveModuleIO swerveModuleIO) {
    super(
        swerveModuleIO,
        SwerveModules.BACK_LEFT,
        new PIDFFController(DriveConstants.Gains.BackLeft.kDefaultDrivingGains),
        new PIDFFController(DriveConstants.Gains.BackLeft.kDefaultAzimuthGains));
  }
}
