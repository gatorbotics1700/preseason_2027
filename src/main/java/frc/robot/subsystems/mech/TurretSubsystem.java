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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  public final TalonFX turretMotor;

  private static TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private final int TURRET_GEARBOX_RATIO = 9;
  private final int GEAR_REVS_PER_TURRET_REV = 6;

  private Rotation2d desiredAngle;

  public TurretSubsystem() {
    turretMotor =
        new TalonFX(
            Constants.TURRET_MOTOR_CAN_ID,
            ""); // TunerConstants.mechCANBus); // TODO change back to mechCANBus for robot
    turretMotor.setNeutralMode(NeutralModeValue.Brake);

    desiredAngle = new Rotation2d(0);

    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG =
        0; // Add 0.2128 V output to overcome gravity (tuned in early feedforward testing)
    slot0Configs.kS =
        0.25; // Add 0.01 V output to overcome static friction (just a guesstimate, but this might
    // just be 0
    slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 2; // A position error of 2.5 rotations results in 12V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // a velocity error of 1 rps results in 0.1 V output

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    turretMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    turretMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
    // Logger.recordOutput("turret/output" + turretMotor.get());
    System.out.println(desiredAngle.getDegrees());
  }

  // TODO need to add something that has the motor switch directions if it reaches a bound (i.e. >
  // 360, )

  public void setDesiredAngle(
      Rotation2d desiredAngle) { // this is for once we start testing targetting
    // this whole method is a mess so TODO check if there's a better way to wrap the angle
    // if (Math.abs(desiredAngle.getRadians()) >= Math.PI) {
    //   // desiredAngle =
    //   //     new Rotation2d(
    //   //         MathUtil.angleModulus(
    //   //             desiredAngle
    //   //                 .getRadians())); // TODO check this - trying to wrap the angle so it stays
    //   // within -180 and 180
    //   int one_eighties =
    //       (int)
    //           (desiredAngle.getRadians()
    //               / Math.PI); // the number of 180 degrees that fit into the angle
    //   if (one_eighties % 2 == 0) {
    //     desiredAngle = new Rotation2d(desiredAngle.getRadians() % Math.PI);
    //   } else {
    //     if (desiredAngle.getRadians() >= Math.PI) {
    //       desiredAngle = new Rotation2d(-((Math.PI) - (desiredAngle.getRadians() % Math.PI)));
    //     }
    //     if (desiredAngle.getRadians() <= -Math.PI) {
    //       desiredAngle = new Rotation2d((Math.PI) + (desiredAngle.getRadians() % Math.PI));
    //     }
    //   }
    // }

    this.desiredAngle = desiredAngle;
  }

  public Rotation2d currentAngle() {
    double motorPositionRevs = turretMotor.getPosition().getValueAsDouble();
    double turretAngleDegrees =
        motorPositionRevs / TURRET_GEARBOX_RATIO / GEAR_REVS_PER_TURRET_REV * 360 % 180;
    return new Rotation2d(
        Math.toRadians(
            turretAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public double degreesToRevs(double turretAngleDegrees) {
    return turretAngleDegrees / 360 * GEAR_REVS_PER_TURRET_REV * TURRET_GEARBOX_RATIO;
  }
}
