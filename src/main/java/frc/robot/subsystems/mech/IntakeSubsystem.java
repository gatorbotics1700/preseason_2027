package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX intakeMotor; // spins the rollers
  public final TalonFX deployMotor; // deploys the entire intake

  private final DigitalInput hallEffect;

  private final TalonFXConfiguration deployTalonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private Rotation2d desiredAngle = new Rotation2d();
  private double desiredIntakeVoltage;
  private double desiredDeployVoltage;

  private static final int DEPLOY_GEARBOX_RATIO = 9; // TODO find the real value
  private static final double DEPLOY_PULLEY_ONE_GEAR_RATIO = 42.0 / 18.0;
  private static final double DEPLOY_PULLEY_TWO_GEAR_RATIO = 36.0 / 18.0;

  public static final double EXTENDED_ANGLE_DEGREES =
      85; // TODO figure out if this is from vertical or from retracted position?
  public static final double RETRACTED_ANGLE_DEGREES = 0; // TODO measure?

  public static final Rotation2d EXTENDED_POSITION =
      new Rotation2d(Math.toRadians(EXTENDED_ANGLE_DEGREES)); // TODO: change
  public static final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(RETRACTED_ANGLE_DEGREES)); // TODO: change

  public static final double HOMING_VOLTAGE = 10; // TODO tune

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.INTAKE_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    deployMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredIntakeVoltage = 0;
    hallEffect =
        new DigitalInput(Constants.INTAKE_HALL_EFFECT_PORT); // TODO:change port during testing

    intakeMotor // TODO see if we actually need to invert
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)));

    deployTalonFXConfigs = new TalonFXConfiguration();

    deployTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive)); // TODO check if we want to invert

    // TODO: TUNE ALL OF THESE
    Slot0Configs slot0Configs = deployTalonFXConfigs.Slot0;

    slot0Configs.kG = 0.2128; // Add _ V output to overcome gravity
    slot0Configs.kS = 0.25; // Add _ V output to overcome static friction
    slot0Configs.kV =
        0.16; // A velocity target of 1 rps results in _ V output (should be somewhere between 0.12
    // and 0.2)
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // a velocity error of 1 rps results in 0.1 V output

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = deployTalonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // 0 gives us unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps, might be 0.12-0.2
    motionMagicConfigs.MotionMagicExpo_kA =
        0.1; // Use a slower kA of 0.1 V/(rps/s) - the larger the kA, the smoother and slower

    deployMotor.getConfigurator().apply(deployTalonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);

    intakeMotor.setVoltage(0);

    // Treat current position as 0 so "deploy position" is the zero reference
    setDeployPositionToZero(); // TODO perhaps get rid of this once we have homing?
  }

  // TODO delete this once we have homing set up
  /**
   * Sets the deploy motor's encoder so the current position is 0. Call when intake is at deploy.
   */
  public void setDeployPositionToZero() {
    deployMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Mech/Intake/Current Deploy Angle", currentAngle());
    Logger.recordOutput("Mech/Intake/Desired Deploy Angle", desiredAngle);
    Logger.recordOutput(
        "Mech/Intake/Desired Deploy Voltage", desiredDeployVoltage); // only gets set for homing
    Logger.recordOutput("Mech/Intake/Current Deploy Motor Output", deployMotor.get());
    Logger.recordOutput("Mech/Intake/Intake Hall Effect", hallEffectTriggered());

    Logger.recordOutput("Mech/Intake/Current Intake Motor Output", intakeMotor.get());
    Logger.recordOutput("Mech/Intake/Desired Intake Voltage", desiredIntakeVoltage);
    // TODO have position and voltage setting in periodic once deploy homing is set up?
  }

  public void retractDeployMotor() {
    deployMotor.setControl(m_request.withPosition(degreesToRevs(RETRACTED_ANGLE_DEGREES)));
  }

  public void extendDeployMotor() {
    deployMotor.setControl(m_request.withPosition(degreesToRevs(EXTENDED_ANGLE_DEGREES)));
  }

  public void setDesiredAngle(Rotation2d angle) {
    if (angle.getDegrees() < RETRACTED_POSITION.getDegrees()) {
      desiredAngle = RETRACTED_POSITION;
    } else if (angle.getDegrees() > EXTENDED_POSITION.getDegrees()) {
      desiredAngle = EXTENDED_POSITION;
    } else {
      desiredAngle = angle;
    }
    deployMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
  }

  public void setDeployVoltage(double voltage) {
    desiredDeployVoltage = voltage;
    deployMotor.setVoltage(desiredDeployVoltage);
  }

  public void setIntakeVoltage(double voltage) {
    desiredIntakeVoltage = voltage;
    intakeMotor.setVoltage(desiredIntakeVoltage);
  }

  public Rotation2d currentAngle() {
    double motorPositionRevs = deployMotor.getPosition().getValueAsDouble();
    double deployAngleDegrees =
        motorPositionRevs
            / DEPLOY_GEARBOX_RATIO
            / DEPLOY_PULLEY_ONE_GEAR_RATIO
            / DEPLOY_PULLEY_TWO_GEAR_RATIO
            * 360.0
            % 360; // TODO check if we multiply or divide by the gear ratio
    return new Rotation2d(
        Math.toRadians(
            deployAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public double degreesToRevs(double deployAngleDegrees) {
    return deployAngleDegrees
        / 360.0
        * DEPLOY_PULLEY_TWO_GEAR_RATIO
        * DEPLOY_PULLEY_ONE_GEAR_RATIO
        * DEPLOY_GEARBOX_RATIO;
  }

  public void zeroIntakeDeploy() {
    double motorPositionRevs = deployMotor.getPosition().getValueAsDouble();
    double offset = degreesToRevs(RETRACTED_POSITION.getDegrees());
    // if we assume the limit switch triggers at the retracted position, then we are calling this
    // method when the current position is the retracted position. therefore we want zero to be
    // wherever we are right now minus the retracted position
    deployMotor.setPosition((motorPositionRevs - offset) % 1);
  }

  public boolean hallEffectTriggered() {
    return hallEffect.get(); // TODO figure out what this actually returns
  }
}
