package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.util.RobotConfigLoader;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

  private final DigitalInput limitSwitch;

  // when false, run voltage control
  private boolean positionControl = true;

  private Rotation2d desiredAngle = HoodConstants.RETRACTED_POSITION;

  private final TalonFX hoodMotor =
      new TalonFX(HoodConstants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  private TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  public HoodSubsystem() {
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    if (RobotConfigLoader.getSerialNumber().equals(RobotConfigLoader.NILE_SERIAL)) {
      talonFXConfigs.withMotorOutput(
          new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    } else {
      talonFXConfigs.withMotorOutput(
          new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    }

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG =
        0.2; // Add 0.2128 V output to overcome gravity (tuned in early feedforward testing)
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

    limitSwitch = new DigitalInput(HoodConstants.HOOD_LIMIT_SWITCH_PORT);
  }

  @Override
  public void periodic() {
    if (limitSwitch.get()) {
      if (positionControl) {
        if (desiredAngle.getDegrees() > getCurrentAngle().getDegrees()) {
          desiredAngle = getCurrentAngle();
          setHoodVoltage(0);
        }
      } else {
        if (hoodMotor.getMotorVoltage().getValueAsDouble() > 0) {
          setHoodVoltage(0);
        }
      }
    }

    if (positionControl) {
      setHoodPosition(desiredAngle);
    }

    Logger.recordOutput("Mech/Hood/Desired Angle", desiredAngle.getDegrees());
    Logger.recordOutput("Mech/Hood/Current Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Hood/Motor Output", hoodMotor.get());
    Logger.recordOutput("Mech/Hood/Current velocity", hoodMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Mech/Hood/Limit switch", isRetractedLimitSwitchPressed());
    Logger.recordOutput(
        "Mech/Climber/Control Mode", positionControl ? "position control" : "voltage control");
  }

  public void setDesiredAngle(Rotation2d desiredAngle) {
    positionControl = true;
    if (desiredAngle.getDegrees() > HoodConstants.RETRACTED_POSITION.getDegrees()) {
      desiredAngle = HoodConstants.RETRACTED_POSITION;
    }
    if (desiredAngle.getDegrees() < HoodConstants.MIN_ANGLE.getDegrees()) {
      desiredAngle = HoodConstants.MIN_ANGLE;
    }
    this.desiredAngle = desiredAngle;
  }

  public Rotation2d getDesiredAngle() {
    return desiredAngle;
  }

  public void setHoodPosition(Rotation2d desiredAngle) {
    positionControl = true;
    hoodMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
  }

  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees
        / 360.0
        * HoodConstants.HOOD_SHAFT_REVS_PER_MECH_REV
        * HoodConstants.HOOD_GEAR_RATIO;
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = hoodMotor.getPosition().getValueAsDouble();
    double hoodAngleDegrees =
        motorPositionRevs
            / HoodConstants.HOOD_GEAR_RATIO
            / HoodConstants.HOOD_SHAFT_REVS_PER_MECH_REV
            * 360
            % 360;
    return new Rotation2d(Math.toRadians(hoodAngleDegrees));
  }

  public void setHoodVoltage(double voltage) {
    positionControl = false;
    hoodMotor.setVoltage(voltage);
  }

  public boolean isRetractedLimitSwitchPressed() {
    return limitSwitch.get();
  }

  public void zeroHood() {
    hoodMotor.setPosition(degreesToRevs(HoodConstants.RETRACTED_POSITION.getDegrees()));
  }
}
