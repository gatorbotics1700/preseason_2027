package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX intakeMotor; // spins the rollers
  public final TalonFX deployMotor; // deploys the entire intake

  // motion magic stuff for deploy NOTE: we will need to figure out some sort of current limit thing
  private final TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;
  // so that something can push the intake back in
  private Rotation2d desiredAngle = new Rotation2d();
  private final double POSITION_DEADBAND_DEGREES = 1; // TODO: tune
  private final int DEPLOY_GEAR_RATIO = 9; // TODO find the real value
  private final double PULLEY_ONE_GEAR_RATIO = 42 / 18;
  private final double PULLEY_TWO_GEAR_RATIO = 36 / 18;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public final Rotation2d EXTENDED_POSITION =
      new Rotation2d(Math.toRadians(90)); // ticks, TODO: change
  public final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(0)); // ticks, TODO: change
  public final double DEADBAND = 2;
  private double intakeSpeed;
  private double intakeVoltage;

  public IntakeSubsystem() {
    intakeMotor =
        new TalonFX(Constants.INTAKE_DEPLOY_MOTOR_CAN_ID, ""); // TunerConstants.mechCANBus);
    deployMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID, ""); // TunerConstants.mechCANBus);
    // TODO check if we really want it inverted because I kinda think we want clockwise to be
    // negative...
    // intakeMotor // TODO see if we actually need to invert
    //     .getConfigurator()
    //     .apply(
    //         new TalonFXConfiguration()
    //             .withMotorOutput(
    //                 new MotorOutputConfigs()
    //                     .withInverted(InvertedValue.CounterClockwise_Positive)));
    // deployMotor // TODO see if we actually need to invert
    //     .getConfigurator()
    //     .apply(
    //         new TalonFXConfiguration()
    //             .withMotorOutput(
    //                 new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    // some motion magic stuff for the deploy motor

    talonFXConfigs = new TalonFXConfiguration();

    // talonFXConfigs.withMotorOutput(
    //     new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

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

    deployMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    // deployMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
    // intakeMotor.setVoltage(intakeVoltage);
  }

  public void setDesiredangle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    this.desiredAngle = desiredAngle;
  }

  public void setIntakeSpeed(double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
  }

  public void setIntakeVoltage(double intakeVoltage) {
    intakeMotor.setVoltage(intakeVoltage);
  }

  public void setDeploySpeed(double speed) {
    deployMotor.setControl(dutyCycleOut.withOutput(speed));
  }

  public Rotation2d currentAngle() {
    double motorPositionTicks = deployMotor.getPosition().getValueAsDouble();
    double deployAngleDegrees =
        motorPositionTicks
            / Constants.KRAKEN_TICKS_PER_REV
            * DEPLOY_GEAR_RATIO
            % 360; // TODO check if we multiply or divide by the gear ratio
    return new Rotation2d(
        Math.toRadians(
            deployAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public double degreesToRevs(double deployAngleDegrees) {
    return deployAngleDegrees
        / 360.0
        * PULLEY_ONE_GEAR_RATIO
        * PULLEY_TWO_GEAR_RATIO
        * DEPLOY_GEAR_RATIO;
  }
}
