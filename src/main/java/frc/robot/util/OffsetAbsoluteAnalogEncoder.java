package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;

public class OffsetAbsoluteAnalogEncoder {
  private static final double MIN_VOLTAGE = 0.0;
  private static final double MAX_VOLTAGE = 4.77;

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
    /*
     adj from -2.2, 2.57
     -2.2 = -180
     2.57 = 180

     (-2.2 - 4.77) / (4.77) * 360


     adj = [-2.2, 2.57]
       ->  [-180, 180]

     y -> degrees
     x -> adj voltage

     (y - y1) = m * (x - x1)
     (180 - -180) = m * (2.57 - -offset)
     m = 75.4717

     y - 180 = 75.4717 * (x + 2.2)
     y = 75.4717 * (x + offset) + 180






    */

    return Rotation2d.fromDegrees(360 / MAX_VOLTAGE * getAdjustedVoltage() - 180);

    // return Rotation2d.fromDegrees(
    //     (getAdjustedVoltage() - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 360.0);
  }
}
