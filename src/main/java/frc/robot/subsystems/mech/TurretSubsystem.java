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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  private final TalonFX turretMotor;

  private static TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 10.0;
  
  private final int TURRET_GEARBOX_RATIO = 9;
  private final int GEAR_REVS_PER_TURRET_REV = 10;
  private final int ENCODER_REVS_PER_TURRET_REV = 10;
  private DutyCycleEncoder boreEncoder =
      new DutyCycleEncoder(TurretConstants.TURRET_BORE_ENCODER_PORT);
  private final DigitalInput hallEffect = new DigitalInput(TurretConstants.TURRET_HALL_EFFECT_PORT);

  private Rotation2d desiredAngle;

  public TurretSubsystem() {
    turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    desiredAngle = getCurrentAngle();

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG =
        0; // Add 0.2128 V output to overcome gravity (tuned in early feedforward testing)
    slot0Configs.kS =
        0.25; // Add 0.01 V output to overcome static friction (just a guesstimate, but this might
    // just be 0
    slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 2; // A position error of 2.5 rotations results in 12V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // a velocity error of 1 rps results in 0.1 V output

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    turretMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    turretMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
    Logger.recordOutput("Mech/Turret/boreEncoder", boreEncoder.get());
    Logger.recordOutput("Mech/Turret/boreEncoder isConnected", boreEncoder.isConnected());
    Logger.recordOutput("Mech/Turret/currentAngle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Turret/desiredAngle", desiredAngle.getDegrees());

    // System.out.println(desiredAngle.getDegrees());
    Logger.recordOutput("Mech/Turret/hallEffect", isHallEffectTriggered());
  }

  public void setDesiredAngle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    double desiredAngleDegrees =
        MathUtil.inputModulus(
            desiredAngle.getDegrees(),
            TurretConstants.MIN_TURRET_ANGLE,
            TurretConstants.MIN_TURRET_ANGLE + 360);
    double alternateDesiredAngleDegrees =
        MathUtil.inputModulus(
            desiredAngleDegrees,
            TurretConstants.MAX_TURRET_ANGLE - 360,
            TurretConstants.MAX_TURRET_ANGLE);
    if (Math.abs(desiredAngleDegrees - getCurrentAngle().getDegrees())
        < Math.abs(alternateDesiredAngleDegrees - getCurrentAngle().getDegrees())) {
      this.desiredAngle = new Rotation2d(Math.toRadians(desiredAngleDegrees));
    } else {
      this.desiredAngle = new Rotation2d(Math.toRadians(alternateDesiredAngleDegrees));
    }
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = turretMotor.getPosition().getValueAsDouble();
    double turretAngleDegrees =
        motorPositionRevs / TURRET_GEARBOX_RATIO / GEAR_REVS_PER_TURRET_REV * 360;
    return new Rotation2d(Math.toRadians(turretAngleDegrees));
  }

  public double degreesToRevs(double turretAngleDegrees) {
    return turretAngleDegrees / 360 * GEAR_REVS_PER_TURRET_REV * TURRET_GEARBOX_RATIO;
  }

  public boolean isHallEffectTriggered() {
    return !hallEffect.get();
  }

  public void setMotorVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  private double getCurrentToOffsetError() {
    return boreEncoder.get() - TurretConstants.TURRET_ENCODER_OFFSET;
  }

  public void homeTurret() {
    turretMotor.setPosition(
        getCurrentToOffsetError()
                / ENCODER_REVS_PER_TURRET_REV
                * TURRET_GEARBOX_RATIO
                * GEAR_REVS_PER_TURRET_REV
            + degreesToRevs(TurretConstants.TURRET_HOMING_ANGLE));
    setDesiredAngle(getCurrentAngle());
  }

  private double getVelocityRadPerSec() {
    double motorRPS = turretMotor.getVelocity().getValueAsDouble();
    return motorRPS / TURRET_GEARBOX_RATIO / GEAR_REVS_PER_TURRET_REV * 2 * Math.PI;
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
            (state) -> Logger.recordOutput("Mech/Turret/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage and logs the motor output
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> turretMotor.setVoltage(voltage.in(Volts)),
            (log) ->
                log.motor("turret")
                    .voltage(Volts.of(turretMotor.getMotorVoltage().getValueAsDouble()))
                    .angularPosition(Radians.of(getCurrentAngle().getRadians()))
                    .angularVelocity(RadiansPerSecond.of(getVelocityRadPerSec())),
            // the subsystem to test (which is us)
            this,
            // name for the task
            "turret");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    return angleDeg >= TurretConstants.MAX_TURRET_ANGLE - SYSID_LIMIT_MARGIN_DEGREES
        || angleDeg <= TurretConstants.MIN_TURRET_ANGLE + SYSID_LIMIT_MARGIN_DEGREES;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .quasistatic(direction)
        .until(this::isSysIdOutOfBounds)
        .withName("Turret SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .dynamic(direction)
        .until(this::isSysIdOutOfBounds)
        .withName("Turret SysId Dynamic " + direction);
  }

}
