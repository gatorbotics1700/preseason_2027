package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class TurretSubsystem extends SubsystemBase {
  public final TalonFX turretMotor;
  // some motion magic stuff here
  private Rotation2d desiredAngle;
  private final double POSITION_DEADBAND_DEGREES = 1; // TODO: tune
  private final int TURRET_GEARBOX_RATIO = 9;
  private final int GEAR_REVS_PER_TURRET_REV = 6;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private static TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  public TurretSubsystem() {
    turretMotor = new TalonFX(Constants.TURRET_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    // TODO: make tuneable constants
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

    turretMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    turretMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
  }

  public void setDesiredAngle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    this.desiredAngle = desiredAngle;
  }

  private void setHoodSpeed(double speed) {
    turretMotor.setControl(dutyCycleOut.withOutput(speed));
  }

  public Rotation2d currentAngle() {
    double motorPositionTicks = turretMotor.getPosition().getValueAsDouble();
    double turretAngleDegrees =
        motorPositionTicks
            / Constants.KRAKEN_TICKS_PER_REV
            * TURRET_GEARBOX_RATIO
            % 360; // TODO check if we multiply or divide by the gear ratio
    return new Rotation2d(
        Math.toRadians(
            turretAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public double degreesToRevs(double turretAngleDegrees) {
    return turretAngleDegrees / 360 * GEAR_REVS_PER_TURRET_REV * TURRET_GEARBOX_RATIO;
  }
}
