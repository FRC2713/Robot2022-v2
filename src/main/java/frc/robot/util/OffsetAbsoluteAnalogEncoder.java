package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class OffsetAbsoluteAnalogEncoder {
  private static final double MIN_VOLTAGE = 0.0;
  private static final double MAX_VOLTAGE = 5.0;

  private AnalogInput analogInput;
  private final double voltageOffset;

  public OffsetAbsoluteAnalogEncoder(int port, double voltageOffset) {
    this.analogInput = new AnalogInput(port);
    this.voltageOffset = voltageOffset;
  }

  public OffsetAbsoluteAnalogEncoder(int port) {
    this(port, 0.0);
  }

  public double getUnadjustedVoltage() {
    return analogInput.getVoltage();
  }

  public double getAdjustedVoltage() {
    return getUnadjustedVoltage() - voltageOffset;
  }

  public Rotation2d getAdjustedRotation2d() {
    return Rotation2d.fromDegrees(
        (getAdjustedVoltage() - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 360.0);
  }
}
