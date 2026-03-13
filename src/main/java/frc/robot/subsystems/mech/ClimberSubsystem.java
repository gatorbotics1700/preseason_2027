package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ClimberSubsystem extends SubsystemBase {
  private boolean positionControl = true; // if false use voltage control

  private final TalonFX motor;
  private final DigitalInput hallEffect;

  private static TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private double desiredPositionInches;
  private static boolean settingPosition;
  private boolean sysIdRunning = false;

  // Tunable PID gains for climber
  public static final LoggedNetworkNumber climberKp =
      new LoggedNetworkNumber("/Tuning/Climber/kP", 4.8);
  public static final LoggedNetworkNumber climberKi =
      new LoggedNetworkNumber("/Tuning/Climber/kI", 0.0);
  public static final LoggedNetworkNumber climberKd =
      new LoggedNetworkNumber("/Tuning/Climber/kD", 0.1);

  private static final double SYSID_LIMIT_MARGIN_INCHES = 1;

  /**
   * Called by SysId commands to indicate test is running; we log voltage/position/velocity in
   * periodic().
   */
  public void setSysIdRunning(boolean running) {
    sysIdRunning = running;
  }

  public ClimberSubsystem() {
    hallEffect = new DigitalInput(ClimberConstants.CLIMBER_HALL_EFFECT_PORT);
    motor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    motor.setNeutralMode(NeutralModeValue.Brake);

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG =
        0.0; // Add 0.2128 V output to overcome gravity (tuned in early feedforward testing)
    slot0Configs.kS =
        0.0; // Add 0.01 V output to overcome static friction (just a guesstimate, but this might
    // just be 0
    slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output

    // Initial PID gains come from tunable LoggedNetworkNumbers
    slot0Configs.kP = climberKp.get(); // A position error of 2.5 rotations results in 12V output
    slot0Configs.kI = climberKi.get(); // no output for integrated error
    slot0Configs.kD = climberKd.get(); // a velocity error of 1 rps results in 0.1 V output

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    motor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  public void periodic() {
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    double newKp = climberKp.get();
    double newKi = climberKi.get();
    double newKd = climberKd.get();
    if (newKp != slot0Configs.kP || newKi != slot0Configs.kI || newKd != slot0Configs.kD) {
      slot0Configs.kP = newKp;
      slot0Configs.kI = newKi;
      slot0Configs.kD = newKd;
      motor.getConfigurator().apply(talonFXConfigs);
    }

    if (!sysIdRunning) {
      if (positionControl
          && (!hallEffectTriggered()
              || desiredPositionInches > ClimberConstants.RETRACTED_HEIGHT_INCHES)) {
        motor.setControl(m_request.withPosition(inchesToRevs(desiredPositionInches)));
        settingPosition = true;
      } else if (hallEffectTriggered()) {
        setClimberVoltage(0);
        settingPosition = false;
      }
    }

    climberLogs();
  }

  public void setDesiredPositionInches(double desiredPositionInches) {
    positionControl = true;
    if (desiredPositionInches < ClimberConstants.RETRACTED_HEIGHT_INCHES) {
      this.desiredPositionInches = ClimberConstants.RETRACTED_HEIGHT_INCHES;
    } else if (desiredPositionInches > ClimberConstants.MAX_EXTENSION_INCHES) {
      this.desiredPositionInches = ClimberConstants.MAX_EXTENSION_INCHES;
    } else {
      this.desiredPositionInches = desiredPositionInches;
    }
  }

  public boolean hallEffectTriggered() {
    return !hallEffect.get();
  }

  public double getCurrentPositionInches() {
    return motor.getPosition().getValueAsDouble()
        / ClimberConstants.CLIMBER_GEAR_RATIO
        * ClimberConstants.WINCH_INCHES_PER_REV;
  }

  public double getDesiredPositionInches() {
    return desiredPositionInches;
  }

  public double inchesToRevs(double positionInches) {
    return positionInches
        / ClimberConstants.WINCH_INCHES_PER_REV
        * ClimberConstants.CLIMBER_GEAR_RATIO;
  }

  public void zeroClimber() {
    // TODO if we decide we want to measure current position from the floor we will need to change
    // this so current position doesn't become zero, but whatever the retracted height is off the
    // floor
    motor.setPosition(inchesToRevs(ClimberConstants.RETRACTED_HEIGHT_INCHES));
  }

  public void setClimberVoltage(double voltage) {
    positionControl = false;
    motor.setVoltage(voltage);
  }

  private double getVelocityInchesPerSec() {
    double motorRPS = motor.getVelocity().getValueAsDouble();
    return motorRPS / ClimberConstants.CLIMBER_GEAR_RATIO * ClimberConstants.WINCH_INCHES_PER_REV;
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
            (state) -> Logger.recordOutput("Mech/Climber/SysID/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage; we log voltage/position/velocity ourselves in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> motor.setVoltage(voltage.in(Volts)),
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "climber");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentPositionInches();
    return angleDeg >= ClimberConstants.MAX_EXTENSION_INCHES - SYSID_LIMIT_MARGIN_INCHES;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .quasistatic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Climber SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .dynamic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Climber SysId Dynamic " + direction);
  }

  public void climberLogs() {
    Logger.recordOutput("Mech/Climber/desiredPositionInches", desiredPositionInches);
    Logger.recordOutput("Mech/Climber/currentPositionInches", getCurrentPositionInches());
    Logger.recordOutput("Mech/Climber/Motor Output", motor.get());
    Logger.recordOutput(
        "Mech/Climber/Control Mode", positionControl ? "position control" : "voltage control");
    Logger.recordOutput(
        "Mech/Climber/Hall Effect Triggered (!halleffect.get)", hallEffectTriggered());
    Logger.recordOutput("Mech/Climber/Setting Positions", settingPosition);

    // SysID
    Logger.recordOutput("Mech/Climber/SysID/climberSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      Logger.recordOutput(
          "Mech/Climber/SysID/climberVoltage", motor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Climber/SysID/climberPosition",
          Units.inchesToMeters(getCurrentPositionInches())); // meters
      Logger.recordOutput(
          "Mech/Climber/SysID/climberVelocity",
          Units.inchesToMeters(1) * getVelocityInchesPerSec()); // m/s
    }
  }
}
