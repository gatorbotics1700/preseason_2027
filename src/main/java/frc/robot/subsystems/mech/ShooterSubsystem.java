package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
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

  public static LoggedNetworkNumber flyWheelSlip =
      new LoggedNetworkNumber("/Tuning/flywheelSlip", 0.27);

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

    leftFlywheelSlot0Configs.kS = 0.25; // Add _ V output to overcome static friction
    leftFlywheelSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12-0.2 V output
    leftFlywheelSlot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    leftFlywheelSlot0Configs.kP = 0.11; // A position error of 1 rps results in 0.11 V output
    leftFlywheelSlot0Configs.kI = 0; // no output for integrated error
    leftFlywheelSlot0Configs.kD = 0; // no output for error derivative

    // MOTION MAGIC EXPO
    MotionMagicConfigs leftMotionMagicConfigs = leftFlywheelTalonFXConfigs.MotionMagic;

    leftMotionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    leftMotionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0/1 seconds)

    rightFlywheelTalonFXConfigs = new TalonFXConfiguration();

    rightFlywheelTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    rightFlywheelSlot0Configs = rightFlywheelTalonFXConfigs.Slot0;

    rightFlywheelSlot0Configs.kS = 0.25; // Add _ V output to overcome static friction
    rightFlywheelSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12-0.2 V output
    rightFlywheelSlot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    rightFlywheelSlot0Configs.kP = 0.11; // A position error of 1 rps results in 0.11 V output
    rightFlywheelSlot0Configs.kI = 0; // no output for integrated error
    rightFlywheelSlot0Configs.kD = 0; // no output for error derivative

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

  public void periodic() {
    Logger.recordOutput("Mech/Shooter/Flywheel Velocity", getFlywheelVelocity());
    Logger.recordOutput("Mech/Shooter/Desired Flywheel Velocity", desiredFlywheelVelocity);

    Logger.recordOutput(
        "Mech/Shooter/Transition Voltage", transitionMotor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("Mech/Shooter/Desired Transition Voltage", desiredTransitionVoltage);
    Logger.recordOutput(
        "Mech/Shooter/Kicker", (transitionMotor.getMotorVoltage().getValueAsDouble() != 0));

    Logger.recordOutput("Mech/Shooter/Should Be Shooting", shouldShoot);

    leftFlywheelMotor.setControl(m_request.withVelocity(desiredFlywheelVelocity));
    rightFlywheelMotor.setControl(m_request.withVelocity(desiredFlywheelVelocity));

    transitionMotor.setVoltage(desiredTransitionVoltage);
  }

  public void setDesiredFlywheelVelocity(double desiredFlywheelVelocity) {
    this.desiredFlywheelVelocity = desiredFlywheelVelocity;
  }

  public double getFlywheelVelocity() {
    return rightFlywheelMotor.getRotorVelocity().getValueAsDouble();
  }

  public void setDesiredTransitionVoltage(double desiredTransitionVoltage) {
    this.desiredTransitionVoltage = desiredTransitionVoltage;
  }

  public double getExitVelocity() {
    return flyWheelSlip.get()
        * getFlywheelVelocity()
        * 2
        * Math.PI
        * ShooterConstants
            .FLYWHEEL_RADIUS_METERS; // 0.7 for the slip is a tentative estimate to account for loss
    // of energy
    // due to
    // energy dissipation/slip. this model assumes that the ball's exit speed matches the wheel's
    // surface speed
  }

  public static double calculateFlywheelSpeed(double shotSpeed) { // shotSpeed in meters/second
    return shotSpeed / flyWheelSlip.get() / 2 / Math.PI / ShooterConstants.FLYWHEEL_RADIUS_METERS;
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

  private double getVelocityRadPerSec() {
    double motorRPS = getFlywheelVelocity();
    return motorRPS / ShooterConstants.FLYWHEEL_GEAR_RATIO * 2 * Math.PI;
  }

  private SysIdRoutine sysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(2),
            // this is the maximum voltage for the test
            Volts.of(4),
            // this is the duration of the test.
            // Note we use `until` when we return the command to abort if we hit turret
            // limits
            Seconds.of(10),
            (state) -> Logger.recordOutput("Mech/Right Shooter/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage and logs the motor output
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> rightFlywheelMotor.setVoltage(voltage.in(Volts)),
            (log) ->
                log.motor("right shooter")
                    .voltage(Volts.of(rightFlywheelMotor.getMotorVoltage().getValueAsDouble()))
                    .angularVelocity(RadiansPerSecond.of(getVelocityRadPerSec())),
            // the subsystem to test (which is us)
            this,
            // name for the task
            "right shooter");
    return new SysIdRoutine(config, mechanism);
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .quasistatic(direction)
        .withName("Turret SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .dynamic(direction)
        .withName("Turret SysId Dynamic " + direction);
  }
}
