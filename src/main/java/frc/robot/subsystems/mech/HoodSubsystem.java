package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.RobotConfigLoader;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

  public static final Rotation2d RETRACTED_POSITION =
      new Rotation2d(
          Math.toRadians(
              RobotConfigLoader.getInt("mech.hood_retracted_degrees"))); // TODO: check number //77
  public static final Rotation2d MAX_EXTENSION =
      new Rotation2d(
          Math.toRadians(
              RobotConfigLoader.getInt(
                  "mech.hood_max_extension_degrees"))); // TODO: check number //57

  private final DigitalInput limitSwitch;
  private boolean wasLimitSwitchPressed = false;

  /** When true, periodic does not run position control; use for retract-to-limit command. */
  private boolean retractingToLimitSwitch = false;

  /** Voltage applied when running toward retract limit (tune sign for your mechanism). */
  public static final double RETRACT_TO_LIMIT_VOLTAGE = -4.0;

  private Rotation2d desiredAngle = RETRACTED_POSITION;
  private final double POSITION_DEADBAND_DEGREES = 1; // TODO: tune

  // GEAR RATIOS
  private static final double HOOD_SHAFT_REVS_PER_MECH_REV =
      RobotConfigLoader.getDouble("mech.hood_shaft_revs_per_mech_rev");
  private static final double HOOD_GEAR_RATIO = RobotConfigLoader.getDouble("mech.hood_gear_ratio");

  private final TalonFX hoodMotor =
      new TalonFX(
          Constants.HOOD_MOTOR_CAN_ID,
          TunerConstants.mechCANBus); // TODO put back mechCANBus on real robot
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  public HoodSubsystem() {
    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    if (RobotConfigLoader.getSerialNumber().equals(RobotConfigLoader.NILE_SERIAL)) {
      talonFXConfigs.withMotorOutput(
          new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    } else {
      talonFXConfigs.withMotorOutput(
          new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    }

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

    hoodMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);

    limitSwitch = new DigitalInput(0); // TODO: change to actual value
    setHoodPositionToRetracted();
  }

  @Override
  public void periodic() {
    if (!retractingToLimitSwitch) {
      setHoodVelocity(desiredAngle);
    }

    if (isRetractedLimitSwitchPressed() && !wasLimitSwitchPressed) {
      setHoodPositionToRetracted();
    }
    wasLimitSwitchPressed = isRetractedLimitSwitchPressed();

    Logger.recordOutput("hood desired angle", desiredAngle.getDegrees());
    Logger.recordOutput("hood motor output", hoodMotor.get());
    Logger.recordOutput("hood current angle", getCurrentAngle().getDegrees());
    System.out.println("HOOD ANGLE: " + getCurrentAngle().getDegrees());
    System.out.println("DESIRED HOOD ANGLE: " + desiredAngle.getDegrees());
    Logger.recordOutput("hood current velocity", hoodMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("hood retract limit switch", isRetractedLimitSwitchPressed());
  }

  public void setDesiredAngle(Rotation2d desiredAngle) {
    // TODO maybe wrap angle like % 360
    // TODO: check this logic -- don't really know whats going on
    if (desiredAngle.getDegrees() > RETRACTED_POSITION.getDegrees()) {
      // the min is
      // negative?
      desiredAngle = RETRACTED_POSITION;
    }
    if (desiredAngle.getDegrees() < MAX_EXTENSION.getDegrees()) {
      desiredAngle = MAX_EXTENSION;
    }
    this.desiredAngle = desiredAngle;
  }

  public void setHoodVelocity(Rotation2d desiredAngle) {
    hoodMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
  }

  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees / 360.0 * HOOD_SHAFT_REVS_PER_MECH_REV * HOOD_GEAR_RATIO;
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = hoodMotor.getPosition().getValueAsDouble();
    double hoodAngleDegrees =
        motorPositionRevs / HOOD_GEAR_RATIO / HOOD_SHAFT_REVS_PER_MECH_REV * 360 % 360;
    return new Rotation2d(
        Math.toRadians(
            hoodAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

  public boolean isRetractedLimitSwitchPressed() {
    return !limitSwitch.get(); // TODO: check this before testing
  }

  public void setHoodPositionToRetracted() {
    hoodMotor.setPosition(degreesToRevs(RETRACTED_POSITION.getDegrees()));
  }

  public void setRetractingToLimitSwitch(boolean retracting) {
    this.retractingToLimitSwitch = retracting;
    if (!retracting) {
      setHoodVoltage(0);
    }
  }
}
