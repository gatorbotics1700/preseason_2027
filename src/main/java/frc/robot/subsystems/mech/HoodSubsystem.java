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
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  public static final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(20)); // TODO find a real number for this
  public final TalonFX hoodMotor;
  // some motion magic stuff here
  private TalonFXConfiguration talonFXConfigs;
  private Rotation2d desiredAngle;
  private final double POSITION_DEADBAND_DEGREES = 0.5; // TODO: tune
  private final int HOOD_GEARBOX_RATIO = 9; // TODO find the real value
  private final int HOOD_SHAFT_REVS_PER_MECH_REV = 155 / 15; // TODO find real value
  private static double currentPositionTicks;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private static MotionMagicExpoVoltage m_request;

  public HoodSubsystem() {
    hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    desiredAngle = new Rotation2d(0);

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    // TODO: make tuneable constants
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    // Add 0.2128 V output to overcome gravity (tuned in early feedforward
    // testing)
    slot0Configs.kG = 0.2128;

    // Add 0.01 V output to overcome static friction (just a guesstimate, but this might just be 0
    slot0Configs.kS = 0.25;

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

    hoodMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    hoodMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));

    Logger.recordOutput("hood desired angle", desiredAngle.getDegrees());
    Logger.recordOutput("hood motor output", hoodMotor.get());
    Logger.recordOutput("hood current angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("hood current velocity", hoodMotor.getVelocity().getValueAsDouble());
  }

  public void setDesiredAngle(Rotation2d desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public void setHoodSpeed(double speed) {
    io.setSpeed(speed);
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = hoodMotor.getPosition().getValueAsDouble();
    double hoodAngleDegrees =
        motorPositionRevs / HOOD_GEARBOX_RATIO / HOOD_SHAFT_REVS_PER_MECH_REV * 360 % 360;
    return new Rotation2d(
        Math.toRadians(
            hoodAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees / 360.0 * HOOD_SHAFT_REVS_PER_MECH_REV * HOOD_GEARBOX_RATIO;
  }

  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }
}
