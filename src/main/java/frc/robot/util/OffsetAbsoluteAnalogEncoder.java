package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class OffsetAbsoluteAnalogEncoder {
  private static final double MIN_VOLTAGE = 0.0;
  private static final double MAX_VOLTAGE = 4.77;

  private final double voltageOffset;
  private AnalogEncoder analogEncoder;

  private LinearFilter offsetStabilizer = LinearFilter.movingAverage(20);

  public OffsetAbsoluteAnalogEncoder(int port, double voltageOffset) {
    this.analogEncoder = new AnalogEncoder(port);
    this.voltageOffset = voltageOffset;
  }

  public OffsetAbsoluteAnalogEncoder(int port) {
    this(port, 0.0);
  }

  public double getUnadjustedVoltage() {
    return analogEncoder.getAbsolutePosition();
  }

  public double getAdjustedVoltage() {
    return analogEncoder.getAbsolutePosition() - voltageOffset;
  }

  public double getFilteredOffset() {
    return offsetStabilizer.calculate(analogEncoder.getAbsolutePosition());
  }

  public Rotation2d getUnadjustedRotation2d() {
    return Rotation2d.fromDegrees((analogEncoder.getAbsolutePosition() - voltageOffset) * 360.0);
  }

  public Rotation2d getAdjustedRotation2d() {
    double unadjustedDegrees = getUnadjustedRotation2d().getDegrees();
    double min = -180, max = 180;

    while (unadjustedDegrees < min) {
      unadjustedDegrees += 360;
    }

    while (unadjustedDegrees >= max) {
      unadjustedDegrees -= 360;
    }

    return Rotation2d.fromDegrees(unadjustedDegrees);
  }
}
