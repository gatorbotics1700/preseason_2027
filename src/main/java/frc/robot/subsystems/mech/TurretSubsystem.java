package frc.robot.subsystems.mech;

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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
  public final TalonFX turretMotor;

  private static TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private final int TURRET_GEARBOX_RATIO = 9;
  private final int GEAR_REVS_PER_TURRET_REV = 10;
  private final int ENCODER_REVS_PER_TURRET_REV = 10;
  private Encoder boreEncoder = new Encoder(7, 3); // TODO real port values
  private final DigitalInput hallEffect = new DigitalInput(5); // TODO real port values
  private final double TURRET_ENCODER_OFFSET = 0.0; // TODO: Find actual offset
  private final double TURRET_HOMING_ANGLE =
      0.0; // TODO: this is the angle for "zeroing" the turret but it might not actually be zero
  private final double TURRET_RANGE_DEGREES = 360; // TODO set actual value
  private final double MIN_TURRET_ANGLE = -180; // TODO: set actual value for min and max
  private final double MAX_TURRET_ANGLE = MIN_TURRET_ANGLE + TURRET_RANGE_DEGREES;

  private Rotation2d desiredAngle;

  public TurretSubsystem() {
    turretMotor =
        new TalonFX(
            Constants.TURRET_MOTOR_CAN_ID,
            TunerConstants.mechCANBus);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    desiredAngle = new Rotation2d(0);

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
    motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    turretMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    turretMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
    // Logger.recordOutput("turret/output" + turretMotor.get());
    // System.out.println(desiredAngle.getDegrees());
    Logger.recordOutput("turret/halleEffect", hallEffect.get());
  }

  public void setDesiredAngle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    this.desiredAngle =
        new Rotation2d(
            Math.toRadians(
                MathUtil.inputModulus(
                    desiredAngle.getDegrees(),
                    MIN_TURRET_ANGLE,
                    MAX_TURRET_ANGLE))); // TODO check this - trying to wrap the angle so it
  }

  public Rotation2d currentAngle() {
    double motorPositionRevs = turretMotor.getPosition().getValueAsDouble();
    double turretAngleDegrees =
        motorPositionRevs / TURRET_GEARBOX_RATIO / GEAR_REVS_PER_TURRET_REV * 360;
    return new Rotation2d(
        Math.toRadians(
            turretAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public double degreesToRevs(double turretAngleDegrees) {
    return turretAngleDegrees / 360 * GEAR_REVS_PER_TURRET_REV * TURRET_GEARBOX_RATIO;
  }

  public boolean getHallEffectValue() {
    return hallEffect.get();
  }

  public void setMotorVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  private double getCurrentToOffsetError() {
    return boreEncoder.get() - TURRET_ENCODER_OFFSET;
  }

  public void homeTurret() {
    turretMotor.setPosition(
        getCurrentToOffsetError()
                / ENCODER_REVS_PER_TURRET_REV
                * TURRET_GEARBOX_RATIO
                * GEAR_REVS_PER_TURRET_REV
            + degreesToRevs(TURRET_HOMING_ANGLE));
  }
}
