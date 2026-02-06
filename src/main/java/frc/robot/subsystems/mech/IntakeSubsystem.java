package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX intakeMotor; // spins the rollers
  public final TalonFX deployMotor; // deploys the entire intake

  // motion magic stuff for deploy NOTE: we will need to figure out some sort of current limit thing
  // so that something can push the intake back in
  private Rotation2d desiredAngle = new Rotation2d();
  private final double POSITION_DEADBAND_DEGREES = 1; // TODO: tune
  private final int DEPLOY_GEAR_RATIO = 9; // TODO find the real value
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public final Rotation2d EXTENDED_POSITION =
      new Rotation2d(Math.toRadians(90)); // ticks, TODO: change
  public final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(0)); // ticks, TODO: change
  public final double DEADBAND = 2;
  private double intakeSpeed;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.INTAKE_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    deployMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    // TODO check if we really want it inverted because I kinda think we want clockwise to be
    // negative...
    intakeMotor // TODO see if we actually need to invert
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    deployMotor // TODO see if we actually need to invert
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    // some motion magic stuff for the deploy motor
  }

  @Override
  public void periodic() {
    // I used a fake pid as a placeholeder, but we should turn to position using motion magic
    double angleError = currentAngle().getDegrees() - desiredAngle.getDegrees();
    if (Math.abs(angleError) > POSITION_DEADBAND_DEGREES) {
      setDeploySpeed(
          0.2 * angleError); // TODO check if this should be -angleError or if I have it backwards
    }

    intakeMotor.setControl(dutyCycleOut.withOutput(intakeSpeed));
  }

  public void setDesiredangle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    this.desiredAngle = desiredAngle;
  }

  public void setIntakeSpeed(double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
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
}
