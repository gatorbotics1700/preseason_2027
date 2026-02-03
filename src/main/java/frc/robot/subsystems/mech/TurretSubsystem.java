package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
  private final int TURRET_GEAR_RATIO = 20; // TODO find the real value
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public TurretSubsystem() {
    turretMotor = new TalonFX(Constants.TURRET_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    // TODO check if we really want it inverted because I kinda think we want clockwise to be
    // negative...
    turretMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
    // some motion magic stuff
  }

  @Override
  public void periodic() {
    // I used a fake pid as a placeholeder, but we should turn to position using motion magic
    double angleError = currentAngle().getDegrees() - desiredAngle.getDegrees();
    if (Math.abs(angleError) > POSITION_DEADBAND_DEGREES) {
      setHoodSpeed(
          0.2 * angleError); // TODO check if this should be -angleError or if I have it backwards
    }
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
            * TURRET_GEAR_RATIO
            % 360; // TODO check if we multiply or divide by the gear ratio
    return new Rotation2d(
        Math.toRadians(
            turretAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }
}
