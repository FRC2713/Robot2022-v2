package frc.robot.subsystems.SwerveIO.module;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.util.OffsetAbsoluteAnalogEncoder;

public class SwerveModuleIOSim implements SwerveModuleIO {

  FlywheelSim azimuthSim = new FlywheelSim(DCMotor.getNEO(1), 7.0 / 150.0, 5);

  public SwerveModuleIOSim(int drivePort, int azimPort, int azimuthEncoderPort, double offset) {

  }

  @Override
  public void updateInputs(SwerveInputs inputs) {
    azimuthSim.update(0.02);
    azimuthSim.getAngularVelocityRPM();
    inputs.aziEncoderRawVolts = azimuthSim.getA;
    inputs.aziEncoderAdjVolts = azimuthSim.getAdjustedVoltage();
    inputs.aziEncoderAdjAngle = azimuthEncoder.getAdjustedRotation2d().getDegrees();
    inputs.outputDriver = driver.getAppliedOutput();
    inputs.aziOutput = azimuth.getAppliedOutput();
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    azimuthSim.setInputVoltage(volts);
  }
}
