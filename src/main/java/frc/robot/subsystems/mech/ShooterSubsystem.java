package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotCalculatorConditions;
import frc.robot.Constants.TunerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX leftFlywheelMotor;
  private final TalonFX rightFlywheelMotor;
  private final TalonFX transitionMotor;

  private double desiredFlywheelVelocity; // in revolutions per second
  private double desiredTransitionVoltage;

  private static MotionMagicVelocityVoltage m_request;

  private static TalonFXConfiguration leftFlywheelTalonFXConfigs;
  private static TalonFXConfiguration rightFlywheelTalonFXConfigs;
  private static TalonFXConfiguration transitionMotorConfigs;

  private static Slot0Configs leftFlywheelSlot0Configs;
  private static Slot0Configs rightFlywheelSlot0Configs;

  private BooleanSupplier shouldShoot;
  private boolean sysIdRunning = false;
  private SysIdRoutine sysIdRoutine;
  private final VoltageOut sysIdVoltageRequest = new VoltageOut(0);
  private static double desiredRotorVelocity = 0;

  public static LoggedNetworkNumber flyWheelSlip =
      new LoggedNetworkNumber("/Tuning/Shooter/flywheelSlip", 0.27);

  // Tunable PID gains for left and right flywheels (NetworkTables-backed)

  public static final LoggedNetworkNumber flywheelKP =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kP", 0.0);
  public static final LoggedNetworkNumber flywheelKI =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kI", 0.0);
  public static final LoggedNetworkNumber flywheelKD =
      new LoggedNetworkNumber("/Tuning/Shooter/Flywheel kD", 0.0);

  public ShooterSubsystem() {
    leftFlywheelMotor =
        new TalonFX(ShooterConstants.LEFT_FLYWHEEL_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    rightFlywheelMotor =
        new TalonFX(ShooterConstants.RIGHT_FLYWHEEL_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    transitionMotor =
        new TalonFX(ShooterConstants.TRANSITION_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredFlywheelVelocity = 0.0;

    // TALONFX & MOTIONMAGIC CONFIGS // TODO everything needs tuning; might not need all values for
    // flywheel
    leftFlywheelTalonFXConfigs = new TalonFXConfiguration();

    leftFlywheelTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    leftFlywheelSlot0Configs = leftFlywheelTalonFXConfigs.Slot0;

    leftFlywheelSlot0Configs.kS = 0.072416; // Add _ V output to overcome static friction
    leftFlywheelSlot0Configs.kV = 0.2553; // A velocity target of 1 rps results in 0.12-0.2 V output
    leftFlywheelSlot0Configs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    // Initial PID gains come from tunable LoggedNetworkNumbers
    leftFlywheelSlot0Configs.kP = flywheelKP.get();
    leftFlywheelSlot0Configs.kI = flywheelKI.get();
    leftFlywheelSlot0Configs.kD = flywheelKD.get();

    // MOTION MAGIC EXPO
    MotionMagicConfigs leftMotionMagicConfigs = leftFlywheelTalonFXConfigs.MotionMagic;

    leftMotionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    leftMotionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0/1 seconds)

    rightFlywheelTalonFXConfigs = new TalonFXConfiguration();

    rightFlywheelTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    rightFlywheelSlot0Configs = rightFlywheelTalonFXConfigs.Slot0;

    rightFlywheelSlot0Configs.kS = 0.072416; // Add _ V output to overcome static friction
    rightFlywheelSlot0Configs.kV =
        0.2553; // A velocity target of 1 rps results in 0.12-0.2 V output
    rightFlywheelSlot0Configs.kA = 0.00; // An acceleration of 1 rps/s requires 0.01 V output
    // Initial PID gains come from tunable LoggedNetworkNumbers
    rightFlywheelSlot0Configs.kP = flywheelKP.get();
    rightFlywheelSlot0Configs.kI = flywheelKI.get();
    rightFlywheelSlot0Configs.kD = flywheelKD.get();

    // MOTION MAGIC EXPO
    MotionMagicConfigs rightMotionMagicConfigs = rightFlywheelTalonFXConfigs.MotionMagic;

    rightMotionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    rightMotionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0/1 seconds)

    transitionMotorConfigs = new TalonFXConfiguration();

    transitionMotorConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    leftFlywheelMotor.getConfigurator().apply(leftFlywheelTalonFXConfigs);
    rightFlywheelMotor.getConfigurator().apply(rightFlywheelTalonFXConfigs);
    transitionMotor.getConfigurator().apply(transitionMotorConfigs);

    m_request = new MotionMagicVelocityVoltage(0);

    shouldShoot =
        () -> {
          return false;
        };
  }

  @Override
  public void periodic() {
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    double newFlywheelKP = flywheelKP.get();
    double newFlywheelKI = flywheelKI.get();
    double newFlywheelKD = flywheelKD.get();
    if (newFlywheelKP != leftFlywheelSlot0Configs.kP
        || newFlywheelKI != leftFlywheelSlot0Configs.kI
        || newFlywheelKD != leftFlywheelSlot0Configs.kD) {
      leftFlywheelSlot0Configs.kP = newFlywheelKP;
      leftFlywheelSlot0Configs.kI = newFlywheelKI;
      leftFlywheelSlot0Configs.kD = newFlywheelKD;
      leftFlywheelMotor.getConfigurator().apply(leftFlywheelTalonFXConfigs);

      rightFlywheelSlot0Configs.kP = newFlywheelKP;
      rightFlywheelSlot0Configs.kI = newFlywheelKI;
      rightFlywheelSlot0Configs.kD = newFlywheelKD;
      rightFlywheelMotor.getConfigurator().apply(rightFlywheelTalonFXConfigs);
    }

    Logger.recordOutput("Mech/Shooter/Flywheel Rotor Velocity", getFlywheelRotorVelocity());
    Logger.recordOutput("Mech/Shooter/Desired Rotor Velocity", desiredRotorVelocity);
    Logger.recordOutput("Mech/Shooter/Desired Flywheel Velocity", desiredFlywheelVelocity);

    Logger.recordOutput(
        "Mech/Shooter/Transition Voltage", transitionMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Mech/Shooter/Desired Transition Voltage", desiredTransitionVoltage);
    Logger.recordOutput(
        "Mech/Shooter/Kicker", (transitionMotor.getMotorVoltage().getValueAsDouble() != 0));

    Logger.recordOutput("Mech/Shooter/Should Be Shooting", shouldShoot);

    Logger.recordOutput("Mech/Shooter/SysID/shooterSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      // Left flywheel
      Logger.recordOutput(
          "Mech/Shooter/SysID/leftShooterVoltage",
          leftFlywheelMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Shooter/SysID/leftShooterPosition",
          leftFlywheelMotor.getPosition().getValueAsDouble()); // rotations
      Logger.recordOutput(
          "Mech/Shooter/SysID/leftShooterVelocity",
          leftFlywheelMotor.getVelocity().getValueAsDouble()
              / ShooterConstants.FLYWHEEL_GEAR_RATIO); // output rot/s
      // Right flywheel
      Logger.recordOutput(
          "Mech/Shooter/SysID/rightShooterVoltage",
          rightFlywheelMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Shooter/SysID/rightShooterPosition",
          rightFlywheelMotor.getPosition().getValueAsDouble()); // rotations
      Logger.recordOutput(
          "Mech/Shooter/SysID/rightShooterVelocity",
          rightFlywheelMotor.getVelocity().getValueAsDouble()
              / ShooterConstants.FLYWHEEL_GEAR_RATIO); // output rot/s
    }
    Logger.recordOutput(
        "Mech/Shooter/Flywheel Position", leftFlywheelMotor.getPosition().getValueAsDouble());
    Logger.recordOutput(
        "Mech/Shooter/Flywheel Voltage", leftFlywheelMotor.getMotorVoltage().getValueAsDouble());

    Logger.recordOutput("Mech/Shooter/MAX SPEED", ShotCalculatorConditions.MAX_SHOT_SPEED);
    Logger.recordOutput(
        "Mech/Shooter/MAX FLYWHEEL SPEED",
        calculateFlywheelSpeed(ShotCalculatorConditions.MAX_SHOT_SPEED));
    Logger.recordOutput(
        "Mech/Shooter/MAX ROTOR SPEED",
        calculateFlywheelSpeed(ShotCalculatorConditions.MAX_SHOT_SPEED)
            / ShooterConstants.FLYWHEEL_GEAR_RATIO);
    // Only control motors if SysID is not running
    if (!sysIdRunning) {
      desiredRotorVelocity = desiredFlywheelVelocity / ShooterConstants.FLYWHEEL_GEAR_RATIO;
      leftFlywheelMotor.setControl(m_request.withVelocity(desiredRotorVelocity));
      rightFlywheelMotor.setControl(m_request.withVelocity(desiredRotorVelocity));
      System.out.println("SETTING ROTOR VELO TO " + desiredRotorVelocity);
    }

    transitionMotor.setVoltage(desiredTransitionVoltage);
  }

  public void setDesiredFlywheelVelocity(double desiredFlywheelVelocity) {
    this.desiredFlywheelVelocity = desiredFlywheelVelocity;
  }

  public double getFlywheelRotorVelocity() {
    return rightFlywheelMotor.getRotorVelocity().getValueAsDouble();
  }

  public void setDesiredTransitionVoltage(double desiredTransitionVoltage) {
    this.desiredTransitionVoltage = desiredTransitionVoltage;
  }

  public double getExitVelocity() {
    return flyWheelSlip.get()
        * getFlywheelRotorVelocity()
        * 2
        * Math.PI
        * ShooterConstants.FLYWHEEL_RADIUS_METERS
        * ShooterConstants
            .FLYWHEEL_GEAR_RATIO; // 0.7 for the slip is a tentative estimate to account for loss
    // of energy
    // due to
    // energy dissipation/slip. this model assumes that the ball's exit speed matches the wheel's
    // surface speed
  }

  public static double calculateFlywheelSpeed(double shotSpeed) { // shotSpeed in meters/second
    return shotSpeed
        / 2.0
        / Math.PI
        / ShooterConstants.FLYWHEEL_RADIUS_METERS
        / ShooterConstants.FLYWHEEL_SLIP;
  }

  public void toggleShouldShoot() {
    if (shouldShoot.getAsBoolean()) {
      shouldShoot =
          () -> {
            return false;
          };
    } else {
      shouldShoot =
          () -> {
            return true;
          };
    }
  }

  /**
   * Sets a supplier for the desired angle. This allows the desired angle to be calculated
   * dynamically each cycle. The supplier will be called each time getDesiredAngle() is called.
   */
  public void setShouldShoot(boolean desiredShouldShootValue) {
    shouldShoot =
        () -> {
          return desiredShouldShootValue;
        };
  }

  /** Returns the desired angle, or null if no angle is set. */
  public BooleanSupplier getShouldShoot() {
    return shouldShoot;
  }

  private void initSysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(1),
            // this is the maximum voltage for the test
            Volts.of(14),
            // this is the duration of the test.
            Seconds.of(15),
            (state) -> Logger.recordOutput("Mech/Shooter/SysID/SysIdState", state.toString()));

    // mechanism for our test. Drives both flywheels; we log voltage/position/velocity for each in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              sysIdRunning = true;
              leftFlywheelMotor.setControl(sysIdVoltageRequest.withOutput(voltage.in(Volts)));
              rightFlywheelMotor.setControl(sysIdVoltageRequest.withOutput(voltage.in(Volts)));
            },
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "shooter");
    System.out.println("CREATING NEW SYSID ROUTINE");
    sysIdRoutine = new SysIdRoutine(config, mechanism);
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    System.out.println("RUNNING SYSID QUASISTATIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .quasistatic(direction)
        .finallyDo(() -> sysIdRunning = false)
        .withName("Flywheel SysId Quasistatic " + direction);
  }

  // measure acceleration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    System.out.println("RUNNING SYSID DYNAMIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .dynamic(direction)
        .finallyDo(() -> sysIdRunning = false)
        .withName("Flywheel SysId Dynamic " + direction);
  }
}
