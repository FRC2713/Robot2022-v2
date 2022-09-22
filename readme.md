# 2022 v2

This is a rewrite of our 2022 robot code, this time exploring a new architecture modeled after team [6328's robot code](https://github.com/Mechanical-Advantage/RobotCode2022), which more easily supports simulation options.

This heavily utilizes [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit) for logging, and [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) for log visualization.

---

### Installation & setup steps

#### Non-driver station laptops

1.  You should install VSCode with WPILib using this guide: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
   - You are free to switch to another IDE (e.g. IntelliJ), but you will be expected to setup/troubleshoot on your own.
2. Download Github Desktop: https://desktop.github.com/
   - Create an account if you do not have one. Tell Justin your username to be added to the 2713 Github organization.
3. Clone (download) this repository using Github Desktop and open in VSCode.
4. Install the Lombok VSCode extension. Click "Extensions" on the left side menu and search "lombok." Install the `Lombok Annotations Support` extension with the pepper icon.
   - Lombok is a Java library used to generate code. It's fairly common within industry.
5. Run a build while connected to the internet.
   - `Ctrl` + `Shift` + `P` -> type `build` -> select `WPILib: Build Robot Code`.
   - Gradle will cache all the third party libraries we use to your computer for a month. If you eventually see an error about not being able to find a class/jar/library, try connecting to the internet and building again.
6. Install AdvantageScope: https://github.com/Mechanical-Advantage/AdvantageScope/releases/latest
   - This is used to view past logs and real-time simulation logs.
7. If you are working with autonomous routines, install PathPlanner: https://github.com/mjansen4857/pathplanner/releases/latest
   - This is used to generate and modify trajectories for the robot to follow.

#### Driver station laptops

Do all of the above, plus:

1. Install the FRC Game Tools from NI: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html
   - You do NOT need a key and you do NOT need to install LabView.
2. Install the REV Hardware Client: https://docs.revrobotics.com/rev-hardware-client/getting-started/installation-instructions
   - This is used to set CAN IDs and update firmware for REV devices (Spark MAXs).
3. Install the Phoenix Framework: https://store.ctr-electronics.com/software/
   - This is used to set CAN IDs and update firmware for CTRE devices (Pigeon 2).

### Terminology

The important terminology to understand is the concept of "inputs" and "outputs." We look at these from the perspective of the robot code.

An *input* is something that is fed into the robot code - typically, a reading from a sensor, like an encoder value, encoder velocity, or limit switch status.

An *output* is something that the robot code feeds into electrical devices - typically, a motor voltage, a PID target, or a solenoid state.

### Goals

Our goal is to abstract away all inputs & outputs in such a way where we can freely swap out hardware and not affect the remainder of the robot code. This is because, in simulation, there is no hardware that we can use, so we must provide differing implementations of "applying our outputs" in order to properly simulate our robot.

### Understanding the architecture

This makes heavy use of java `interface`s. Take this `DriveIO` interface example:

```java
public interface DriveIO {
    public void setVoltages(double leftVolts, double rightVolts);
    public void setVelocity(double leftVelocity, double rightVelocity);
}
```

This interface describes the most basic possible drivetrain - the drivetrain code will have 2 possible outputs - it will either output a pair of voltages (1 for each side of the drivetrain), or a pair of target velocities (also 1 for each side of the drivetrain).

Lets suppose we want to *implement* a drivetrain with Spark MAXs.

```java
public class DriveIOSparkMAX implements DriveIO {
    CANSparkMAX leftLeader = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMAX rightLeader = new CANSparkMax(1, MotorType.kBrushless);
    public DriveIOSparkMAX() {}

    @Override
    public void setVoltages(double leftVolts, double rightVolts) {
        // This is an implementation of a DriveIO method.
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    @Override
    public void setVelocity(double leftVelocity, double rightVelocity) {
        // This is an implementation of a DriveIO method.
        leftLeader.getPIDController().setReference(leftVelocity, ControlType.kVelocity);
        rightLeader.getPIDController().setReference(rightVelocity, ControlType.kVelocity);
    }
}
```

We can then create a drivetrain in `Robot.java` like so:

```java
DriveIO drive = new DriveIOSparkMAX();
```

Now, lets say we want to simulate the drivetrain. We can provide an *alternate* implementation of `DriveIO` that interacts with WPILib's drivetrain simulator classes:

```java
public class DriveIOSim implements DriveIO {
    DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);
    private PIDController leftPID = new PIDController(0.0, 0.0, 0.0);
    private PIDController rightPID = new PIDController(0.0, 0.0, 0.0);

    public DriveIOSim() {}


    @Override
    public void setVoltages(double leftVolts, double rightVolts) {
        // This is an implementation of a DriveIO method.
        sim.setInputs(leftVolts, rightVolts);
    }

    @Override
    public void setVelocity(double leftVelocity, double rightVelocity) {
        // This is an implementation of a DriveIO method.
        leftPID.setSetpoint(leftVelocity);
        rightPID.setSetpoint(rightVelocity);
        // Some other stuff needed here, but omitted for brevity.
    }
}
```

Now, if we want to simulate it, we change our `Robot.java`:

```java
DriveIO drive = new DriveIOSim();
```

The remainder of the robot code does not care which implementation of `DriveIO` exists, as long as one of them does, since all of them will implement `setVoltages` and `setVelocity`.

Better yet, we can automatically decide which `DriveIO` implementation to use:

```java
DriveIO drive;
if (isReal()) {
    drive = new DriveIOSparkMAX();
} else {
    drive = new DriveIOSim();
}
```

Or, in a single line, using a [ternary operator](https://www.geeksforgeeks.org/java-ternary-operator-with-examples):

```java
DriveIO drive = isReal() ? new DriveIOSparkMAX() : new DriveIOSim();
```
