package frc.robot.subsystems.SwerveIO.module.modules;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveIO.module.SwerveModule;
import frc.robot.subsystems.SwerveIO.module.SwerveModuleIO;
import frc.robot.subsystems.SwerveIO.module.SwerveModules;
import frc.robot.util.PIDFFController;

public class BackRightModule extends SwerveModule {
  public BackRightModule(SwerveModuleIO swerveModuleIO) {
    super(
        swerveModuleIO,
        SwerveModules.BACK_RIGHT,
        new PIDFFController(DriveConstants.Gains.BackRight.kDefaultDrivingGains),
        new PIDFFController(DriveConstants.Gains.BackRight.kDefaultAzimuthGains));
  }
}
