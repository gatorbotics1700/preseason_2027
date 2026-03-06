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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private boolean positionControl = true; // if false use voltage control

  private final TalonFX motor;
  private final DigitalInput limitSwitch;

  private static TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private double desiredPositionInches;

  private static final double SYSID_LIMIT_MARGIN_INCHES = 1;

  public ClimberSubsystem() {
    limitSwitch = new DigitalInput(ClimberConstants.CLIMBER_LIMIT_SWITCH_PORT);
    motor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    motor.setNeutralMode(NeutralModeValue.Brake);

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG =
        0.2128; // Add 0.2128 V output to overcome gravity (tuned in early feedforward testing)
    slot0Configs.kS =
        0.25; // Add 0.01 V output to overcome static friction (just a guesstimate, but this might
    // just be 0
    slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // a velocity error of 1 rps results in 0.1 V output

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    motor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  public void periodic() {
    Logger.recordOutput("Mech/Climber/desiredPositionInches", desiredPositionInches);
    Logger.recordOutput("Mech/Climber/currentPositionInches", getCurrentPositionInches());
    Logger.recordOutput("Mech/Climber/Motor Output", motor.get());
    Logger.recordOutput(
        "Mech/Climber/Control Mode", positionControl ? "position control" : "voltage control");
    Logger.recordOutput("Mech/Climber/Limit Switch", limitSwitchPressed());
    if (!limitSwitchPressed() && positionControl) {
      motor.setControl(m_request.withPosition(inchesToRevs(desiredPositionInches)));
    } else {
      setClimberVoltage(0); // TODO figure out if this actually works?
    }
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

  public boolean limitSwitchPressed() {
    return limitSwitch
        .get(); // TODO confirm that normally closed limit switch is true when pressed?
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
    motor.setVoltage(voltage); // TODO figure out if this actually works?
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
            (state) -> Logger.recordOutput("Mech/Climber/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage and logs the motor output
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> motor.setVoltage(voltage.in(Volts)),
            (log) ->
                log.motor("climber")
                    .voltage(Volts.of(motor.getMotorVoltage().getValueAsDouble()))
                    .linearPosition(
                        Meters.of(
                            getCurrentPositionInches())) // TODO the linear position and linear
                    // velocity return in units with metters,
                    // check if the Meters.of part does
                    // conversion
                    .linearVelocity(MetersPerSecond.of(getVelocityInchesPerSec())),
            // the subsystem to test (which is us)
            this,
            // name for the task
            "climber");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentPositionInches();
    return angleDeg >= ClimberConstants.MAX_EXTENSION_INCHES - SYSID_LIMIT_MARGIN_INCHES;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .quasistatic(direction)
        .until(this::isSysIdOutOfBounds)
        .withName("Climber SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .dynamic(direction)
        .until(this::isSysIdOutOfBounds)
        .withName("Climber SysId Dynamic " + direction);
  }
}
