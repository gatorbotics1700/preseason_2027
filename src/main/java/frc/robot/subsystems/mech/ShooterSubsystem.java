package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  public static final double TRANSITION_VOLTAGE = 0;
  public static final double FLYWHEEL_SPEED_DEADBAND = 0.1;
  public static final double FLYWHEEL_GEAR_RATIO = 30.0 / 14.0;
  private final TalonFX flywheelMotor;
  private final TalonFX transitionMotor;
  private double desiredFlywheelVelocity; // in revolutions per second
  private double desiredTransitionVoltage;
  private static MotionMagicVelocityVoltage m_request;
  private static TalonFXConfiguration flywheelTalonFXConfigs;
  private static Slot0Configs flywheelSlot0Configs;
  private static final double FLYWHEEL_RADIUS = 0.0508;
  private boolean shouldShoot;

  public ShooterSubsystem() {
    // TODO put mech canbus id for real robot
    flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID, ""); // TunerConstants.mechCANBus);
    transitionMotor =
        new TalonFX(Constants.TRANSITION_MOTOR_CAN_ID, ""); // TunerConstants.mechCANBus);

    setShooterVoltages(0, 0);

    desiredFlywheelVelocity = 0.0;

    // TALONFX & MOTIONMAGIC CONFIGS // TODO everything needs tuning; might not need all values for
    // flywheel
    flywheelTalonFXConfigs = new TalonFXConfiguration();

    flywheelTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    // TODO: make tuneable constants
    flywheelSlot0Configs = flywheelTalonFXConfigs.Slot0;

    flywheelSlot0Configs.kS = 0.25; // Add _ V output to overcome static friction
    flywheelSlot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12-0.2 V output
    flywheelSlot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    flywheelSlot0Configs.kP = 0.11; // A position error of 1 rps results in 0.11 V output
    flywheelSlot0Configs.kI = 0; // no output for integrated error
    flywheelSlot0Configs.kD = 0; // no output for error derivative

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = flywheelTalonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0/1 seconds)

    flywheelMotor.getConfigurator().apply(flywheelTalonFXConfigs);

    m_request = new MotionMagicVelocityVoltage(0);

    shouldShoot = false;
  }

  public void periodic() {
    flywheelMotor.setControl(m_request.withVelocity(desiredFlywheelVelocity));

    transitionMotor.setVoltage(desiredTransitionVoltage);

    Logger.recordOutput(
        "flywheel current velocity", flywheelMotor.getVelocity().getValueAsDouble());

    if (transitionMotor.getMotorVoltage().getValueAsDouble() != 0) {
      Logger.recordOutput("Kicker", true);
    } else {
      Logger.recordOutput("Kicker", false);
    }
  }

  public void setFlywheelVelocity(double desiredFlywheelVelocity) {
    this.desiredFlywheelVelocity = desiredFlywheelVelocity;
  }

  public double getFlywheelVelocity() {
    return flywheelMotor
        .getRotorVelocity()
        .getValueAsDouble(); // TODO figure out if this is the right method
  }

  public void setDesiredTransitionVoltage(double desiredTransitionVoltage) {
    this.desiredTransitionVoltage = desiredTransitionVoltage;
  }

  public void setShooterVoltages(double flywheelVoltage, double transitionVoltage) {
    flywheelMotor.setVoltage(flywheelVoltage);
    Logger.recordOutput("flywheelMotor velocity", flywheelMotor.getVelocity().getValueAsDouble());
    transitionMotor.setVoltage(transitionVoltage);
  }

  public void setFlywheelVoltage(double voltage) {
    flywheelMotor.setVoltage(voltage);
  }

  public void setTransitionVoltage(double voltage) {
    transitionMotor.setVoltage(voltage);
  }

  public double getExitVelocity() {
    return 0.7
        * getFlywheelVelocity()
        * 2
        * Math.PI
        * Constants
            .FLYWHEEL_RADIUS_METERS; // 0.7 is a tentative estimate to account for loss of energy
    // due to
    // energy dissipation/slip. this model assumes that the ball's exit speed matches the wheel's
    // surface speed
  }

  public static double calculateFlywheelSpeed(double shotSpeed) {
    return shotSpeed / FLYWHEEL_RADIUS;
  }

  public boolean getShouldShoot() {
    return shouldShoot;
  }

  public void toggleShouldShoot() {
    shouldShoot = !shouldShoot;
  }
}
