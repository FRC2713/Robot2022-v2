package frc.robot.subsystems.SwerveIO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveIO {
    public static class SwerveInputs implements LoggableInputs {
        public double gyroYawPosition = 0.0;
        public double gyroPitchPosition = 0.0;
        public double gyroRollPosition = 0.0;
        public double gyroCompassHeading = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("Gyro/Yaw", gyroYawPosition);
            table.put("Gyro/Pitch", gyroPitchPosition);
            table.put("Gyro/Roll", gyroRollPositionRadians);
            table.put("Gyro/Compass Heading", gyroCompassHeading);
        }

        @Override
        public void fromLog(LogTable table) {
            gyroYawPosition = table.getDouble("Gyro/Yaw", gyroYawPosition);
            gyroPitchPosition = table.getDouble("Gyro/Pitch", gyroPitchPosition);
            gyroRollPosition = table.getDouble("Gyro/Roll", gyroRollPosition);
            gyroCompassHeading = table.getDouble("Gyro/Compass Heading", gyroCompassHeading);
        }
    }

    public void updateInputs(SwerveInputs inputs);
}