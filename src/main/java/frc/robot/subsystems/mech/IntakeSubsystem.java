package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor; // spins the rollers
  private final TalonFX deployMotor; // deploys the entire intake

  private final DigitalInput hallEffect;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 3;

  private final TalonFXConfiguration deployTalonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private Rotation2d desiredAngle = new Rotation2d();
  private double desiredIntakeVoltage;
  private double desiredDeployVoltage;
  private BooleanSupplier isDeployed;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    deployMotor =
        new TalonFX(IntakeConstants.INTAKE_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredIntakeVoltage = 0;
    hallEffect =
        new DigitalInput(
            IntakeConstants.INTAKE_HALL_EFFECT_PORT); // TODO:change port during testing

    intakeMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)));

    deployTalonFXConfigs = new TalonFXConfiguration();

    deployTalonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)); // TODO check if we want to invert

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

    isDeployed =
        () -> {
          return false;
        };
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Mech/Intake/Current Deploy Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Intake/Desired Deploy Angle", desiredAngle.getDegrees());
    Logger.recordOutput(
        "Mech/Intake/Desired Deploy Voltage", desiredDeployVoltage); // only gets set for homing
    Logger.recordOutput("Mech/Intake/Current Deploy Motor Output", deployMotor.get());
    Logger.recordOutput("Mech/Intake/Intake Hall Effect", isHallEffectTriggered());
    Logger.recordOutput("Mech/Intake/IsDeployed", isDeployed.getAsBoolean());

    Logger.recordOutput("Mech/Intake/Current Intake Motor Output", intakeMotor.get());
    Logger.recordOutput("Mech/Intake/Desired Intake Voltage", desiredIntakeVoltage);
    // TODO have position and voltage setting in periodic once deploy homing is set up?
  }

  public void retractDeployMotor() {
    deployMotor.setControl(
        m_request.withPosition(degreesToRevs(IntakeConstants.RETRACTED_ANGLE_DEGREES)));
  }

  public void extendDeployMotor() {
    deployMotor.setControl(
        m_request.withPosition(degreesToRevs(IntakeConstants.EXTENDED_ANGLE_DEGREES)));
  }

  public void setDesiredAngle(Rotation2d angle) {
    if (angle.getDegrees() < IntakeConstants.RETRACTED_POSITION.getDegrees()) {
      desiredAngle = IntakeConstants.RETRACTED_POSITION;
    } else if (angle.getDegrees() > IntakeConstants.EXTENDED_POSITION.getDegrees()) {
      desiredAngle = IntakeConstants.EXTENDED_POSITION;
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

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = deployMotor.getPosition().getValueAsDouble();
    double deployAngleDegrees =
        motorPositionRevs
            / IntakeConstants.DEPLOY_GEARBOX_RATIO
            / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
            / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
            * 360.0
            % 360; // TODO check if we multiply or divide by the gear ratio
    return new Rotation2d(Math.toRadians(deployAngleDegrees));
  }

  public double degreesToRevs(double deployAngleDegrees) {
    return deployAngleDegrees
        / 360.0
        * IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        * IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        * IntakeConstants.DEPLOY_GEARBOX_RATIO;
  }

  public void zeroIntakeDeploy() {
    deployMotor.setPosition(IntakeConstants.RETRACTED_POSITION.getDegrees());
  }

  public boolean isHallEffectTriggered() {
    return !hallEffect.get(); // TODO figure out what this actually returns
  }

  public void toggleIntake() {
    if (isDeployed.getAsBoolean()) {
      isDeployed =
          () -> {
            return false;
          };
    } else {
      isDeployed =
          () -> {
            return true;
          };
    }
  }

  public BooleanSupplier getIsDeployed() {
    return isDeployed;
  }

  private double getVelocityRadPerSec(){
    double motorRPS = deployMotor.getVelocity().getValueAsDouble();
    return motorRPS / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO / IntakeConstants.DEPLOY_GEARBOX_RATIO * 2 * Math.PI;
  }

  private SysIdRoutine sysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(2),
            // this is the maximum voltage for the test
            Volts.of(4),
            // this is the duration of the test.
            // Note we use `until` when we return the command to abort if we hit turret
            // limits
            Seconds.of(10),
            (state) -> Logger.recordOutput("Mech/Turret/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage and logs the motor output
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> deployMotor.setVoltage(voltage.in(Volts)),
            (log) ->
                log.motor("turret")
                    .voltage(Volts.of(deployMotor.getMotorVoltage().getValueAsDouble()))
                    .angularPosition(Radians.of(getCurrentAngle().getRadians()))
                    .angularVelocity(RadiansPerSecond.of(getVelocityRadPerSec())),
            // the subsystem to test (which is us)
            this,
            // name for the task
            "turret");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    return angleDeg >= IntakeConstants.EXTENDED_ANGLE_DEGREES - SYSID_LIMIT_MARGIN_DEGREES
      || angleDeg <= IntakeConstants.RETRACTED_ANGLE_DEGREES + SYSID_LIMIT_MARGIN_DEGREES;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .quasistatic(direction)
        .until(this::isSysIdOutOfBounds)
        .withName("Turret SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine()
        .dynamic(direction)
        .until(this::isSysIdOutOfBounds)
        .withName("Turret SysId Dynamic " + direction);
  }
}
