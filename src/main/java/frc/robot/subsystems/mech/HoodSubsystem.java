package frc.robot.subsystems.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mech.MechIOs.HoodIO;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  public static final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(0)); // TODO: find a real number

  public HoodSubsystem() {
    hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    desiredAngle = new Rotation2d(0);

  private Rotation2d desiredAngle = RETRACTED_POSITION;
  private final double POSITION_DEADBAND_DEGREES = 1; // TODO: tune
  private static final double HOOD_SHAFT_REVS_PER_MECH_REV = 155 / 15.0;
  private static final double HOOD_GEARBOX_RATIO = 9.0;
  private double desiredSpeed;

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
    setHoodSpeed(desiredAngle);

    Logger.recordOutput("hood desired angle", desiredAngle.getDegrees());
    // Logger.recordOutput("hood motor output", hoodMotor.get());
    // Logger.recordOutput("hood current angle", getCurrentAngle().getDegrees());
    // Logger.recordOutput("hood current velocity", hoodMotor.getVelocity().getValueAsDouble());
    io.updateInputs(inputs);
    //  Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/Velocity", inputs.velocityRevsPerSec);
    Logger.recordOutput("Hood/position", inputs.positionRevs);
    Logger.recordOutput("Hood/DesiredSpeed", desiredSpeed);
  }

  public void setDesiredAngle(Rotation2d desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public void setHoodSpeed(Rotation2d desiredAngle) {
    io.setSpeed(desiredAngle);
  }

  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees / 360.0 * HOOD_SHAFT_REVS_PER_MECH_REV * HOOD_GEARBOX_RATIO;
  }
}
