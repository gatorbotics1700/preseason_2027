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
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX intakeMotor; // spins the rollers
  public final TalonFX deployMotor; // deploys the entire intake

  private final DigitalInput limitSwitch;
  private final DigitalInput hallEffect;

  private final TalonFXConfiguration deployTalonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private Rotation2d desiredAngle = new Rotation2d();
  private double desiredVoltage;

  private final int DEPLOY_GEARBOX_RATIO = 9; // TODO find the real value
  private final double DEPLOY_PULLEY_ONE_GEAR_RATIO = 42.0 / 18.0;
  private final double DEPLOY_PULLEY_TWO_GEAR_RATIO = 36.0 / 18.0;

  public final Rotation2d EXTENDED_POSITION = new Rotation2d(Math.toRadians(85)); // TODO: change
  public final Rotation2d RETRACTED_POSITION = new Rotation2d(Math.toRadians(0)); // TODO: change

  public IntakeSubsystem() {
    // TODO change back to mechCANbus for robot
    intakeMotor =
        new TalonFX(Constants.INTAKE_DEPLOY_MOTOR_CAN_ID, ""); // TunerConstants.mechCANBus);
    deployMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID, ""); // TunerConstants.mechCANBus);

    desiredVoltage = 0;

    limitSwitch = new DigitalInput(0); // TODO: change during testing
    hallEffect = new DigitalInput(1);//TODO:change port during testing

    intakeMotor // TODO see if we actually need to invert
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)));

    deployTalonFXConfigs = new TalonFXConfiguration();

    deployTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

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
    setDeployPositionToZero();
  }

  /**
   * Sets the deploy motor's encoder so the current position is 0. Call when intake is at deploy.
   */
  public void setDeployPositionToZero() {
    deployMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    // TODO uncomment out this code when ready to test without voltage instant commands
    Logger.recordOutput("Intake/Deploy Limit Switch", limitSwitch.get());
    Logger.recordOutput("Intake/Current Deploy Angle", currentAngle());
    if (!limitSwitch.get()) { // TODO: check this before testing
      deployMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
    } else {
      deployMotor.setControl(m_request.withPosition(degreesToRevs(currentAngle().getDegrees())));
    }
    if(hallEffect.get()){//TODO: check closed vs open before testing
      setDeployPositionToZero();
    }
    // intakeMotor.setVoltage(desiredVoltage);
  }

  public void setDesiredAngle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    this.desiredAngle = desiredAngle;
  }

  public void setDesiredIntakeVoltage(double desiredIntakeVoltage) {
    this.desiredVoltage = desiredIntakeVoltage;
  }

  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
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
}
