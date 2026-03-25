package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TunerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor; // spins the rollers
  private final TalonFX deployMotor; // deploys the entire intake

  private final DigitalInput hallEffect;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 3;
  private boolean sysIdRunning = false;

  private final TalonFXConfiguration deployTalonFXConfigs;

  private static Slot0Configs slot0Configs;
  private static MotionMagicConfigs motionMagicConfigs;
  private static CurrentLimitsConfigs currentLimitConfigs;

  /**
   * Called by SysId commands to indicate test is running; we log voltage/position/velocity in
   * periodic().
   */
  public void setSysIdRunning(boolean running) {
    sysIdRunning = running;
  }

  private static MotionMagicExpoVoltage m_request;

  // Tunable PID gains for intake deploy
  public static final LoggedNetworkNumber intakeKp =
      new LoggedNetworkNumber("/Tuning/Intake/kP", 4.4);
  public static final LoggedNetworkNumber intakeKi =
      new LoggedNetworkNumber("/Tuning/Intake/kI", 0.0);
  public static final LoggedNetworkNumber intakeKd =
      new LoggedNetworkNumber("/Tuning/Intake/kD", 0.1);

  // Tunable Feedfoward gains for intake deploy
  public static final LoggedNetworkNumber intakeKg =
      new LoggedNetworkNumber("/Tuning/Intake/kG", 0.0);
  public static final LoggedNetworkNumber intakeKs =
      new LoggedNetworkNumber("/Tuning/Intake/kS", 0.0);
  public static final LoggedNetworkNumber intakeKv =
      new LoggedNetworkNumber("/Tuning/Intake/kV", 0.16);
  public static final LoggedNetworkNumber intakeKa =
      new LoggedNetworkNumber("/Tuning/Intake/kA", 0.01);

  public static final LoggedNetworkNumber intakeExpoKa =
      new LoggedNetworkNumber("/Tuning/Intake/Expo kA", 0.16);
  public static final LoggedNetworkNumber intakeExpoKv =
      new LoggedNetworkNumber("/Tuning/Intake/Expo kV", 0.1);

  private Rotation2d desiredAngle = new Rotation2d();
  private boolean useDeployPositionControl = false;
  private double desiredIntakeVoltage;
  private double desiredDeployVoltage;
  private BooleanSupplier isDeployed;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    deployMotor =
        new TalonFX(IntakeConstants.INTAKE_DEPLOY_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredIntakeVoltage = 0;
    hallEffect = new DigitalInput(IntakeConstants.INTAKE_HALL_EFFECT_PORT);

    intakeMotor
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
    slot0Configs = deployTalonFXConfigs.Slot0;

    slot0Configs.kG = intakeKg.get(); // 0.2128 vaguely works
    slot0Configs.kS = intakeKs.get(); // 0.25 vaguely works
    slot0Configs.kV = intakeKv.get(); // 0.16 vaguely works
    slot0Configs.kA = intakeKa.get(); // 0.01 vaguely works

    // Initial PID gains come from tunable LoggedNetworkNumbers
    slot0Configs.kP = intakeKp.get(); // 1 vaguely works
    slot0Configs.kI = intakeKi.get(); // no output for integrated error
    slot0Configs.kD = intakeKd.get(); // 0.1 vaguely works

    // MOTION MAGIC EXPO
    motionMagicConfigs = deployTalonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // 0 gives us unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV =
        intakeExpoKv.get(); // was 0.16 kV is around 0.12 V/rps, might be 0.12-0.2
    motionMagicConfigs.MotionMagicExpo_kA =
        intakeExpoKa
            .get(); // was 0.1 Use a slower kA of 0.1 V/(rps/s) - the larger the kA, the smoother
    // and slower

    currentLimitConfigs = deployTalonFXConfigs.CurrentLimits;
    currentLimitConfigs.StatorCurrentLimit = 10;

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
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    updateSlot0Configs();
    updateMotionMagicConfigs();

    if (!sysIdRunning && useDeployPositionControl) {
      deployMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
      //   if (isDeployed.getAsBoolean()
      //       && currentLimitConfigs.StatorCurrentLimit != 20
      //       && getCurrentAngle().getDegrees() > 30) {
      //     currentLimitConfigs.StatorCurrentLimit = 20;
      //     deployMotor.getConfigurator().apply(deployTalonFXConfigs);
      //   } else if ((!isDeployed.getAsBoolean() || getCurrentAngle().getDegrees() < 30)
      //       && currentLimitConfigs.StatorCurrentLimit != 60) {
      //     currentLimitConfigs.StatorCurrentLimit = 60;
      //     deployMotor.getConfigurator().apply(deployTalonFXConfigs);
      //   }
    }

    intakeLogs();
  }

  public void retractDeployMotor() {
    setDesiredAngle(IntakeConstants.RETRACTED_POSITION);
  }

  public void extendDeployMotor() {
    setDesiredAngle(IntakeConstants.EXTENDED_POSITION);
  }

  public void setDesiredAngle(Rotation2d angle) {
    useDeployPositionControl = true;
    if (angle.getDegrees() < IntakeConstants.RETRACTED_POSITION.getDegrees()) {
      desiredAngle = IntakeConstants.RETRACTED_POSITION;
    } else if (angle.getDegrees() > IntakeConstants.EXTENDED_POSITION.getDegrees()) {
      desiredAngle = IntakeConstants.EXTENDED_POSITION;
    } else {
      desiredAngle = angle;
    }
  }

  /** Returns the stored desired angle (always the true target, independent of deadband). */
  public Rotation2d getDesiredAngle() {
    return desiredAngle;
  }

  public void setDeployVoltage(double voltage) {
    useDeployPositionControl = false;
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
            % 360;
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
    return !hallEffect.get();
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

  public void setIsDeployedToTrue() {
    isDeployed =
        () -> {
          return true;
        };
  }

  public void setIsDeployedToFalse() {
    isDeployed =
        () -> {
          return false;
        };
  }

  private double getVelocityRadPerSec() {
    double motorRPS = deployMotor.getVelocity().getValueAsDouble();
    return motorRPS
        / IntakeConstants.DEPLOY_PULLEY_ONE_GEAR_RATIO
        / IntakeConstants.DEPLOY_PULLEY_TWO_GEAR_RATIO
        / IntakeConstants.DEPLOY_GEARBOX_RATIO
        * 2
        * Math.PI;
  }

  private SysIdRoutine sysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(2),
            // this is the maximum voltage for the test
            Volts.of(18),
            // this is the duration of the test.
            // Note we use `until` when we return the command to abort if we hit intake deployed or
            // retracted positions
            Seconds.of(10),
            (state) -> Logger.recordOutput("Mech/Intake/SysID/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage; we log voltage/position/velocity ourselves in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> deployMotor.setVoltage(voltage.in(Volts)),
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "intake");
    return new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    boolean isSysIdOutOfBounds =
        angleDeg >= IntakeConstants.EXTENDED_ANGLE_DEGREES + SYSID_LIMIT_MARGIN_DEGREES
            || angleDeg <= IntakeConstants.RETRACTED_ANGLE_DEGREES - SYSID_LIMIT_MARGIN_DEGREES;
    Logger.recordOutput("Mech/Intake/SysID/Outofbounds", isSysIdOutOfBounds);

    return isSysIdOutOfBounds;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .quasistatic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Intake SysId Quasistatic " + direction);
  }

  // measure accelaration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setSysIdRunning(true))
        .andThen(
            sysIdRoutine()
                .dynamic(direction)
                .until(this::isSysIdOutOfBounds)
                .finallyDo(() -> setSysIdRunning(false)))
        .withName("Intake SysId Dynamic " + direction);
  }

  public void updateSlot0Configs() {
    double newKp = intakeKp.get();
    double newKi = intakeKi.get();
    double newKd = intakeKd.get();
    double newKg = intakeKg.get();
    double newKs = intakeKs.get();
    double newKa = intakeKa.get();
    double newKv = intakeKv.get();

    if (newKp != slot0Configs.kP
        || newKi != slot0Configs.kI
        || newKd != slot0Configs.kD
        || newKa != slot0Configs.kA
        || newKv != slot0Configs.kV
        || newKg != slot0Configs.kG
        || newKs != slot0Configs.kS) {

      slot0Configs.kP = newKp;
      slot0Configs.kI = newKi;
      slot0Configs.kD = newKd;
      slot0Configs.kG = newKg;
      slot0Configs.kS = newKs;
      slot0Configs.kA = newKa;
      slot0Configs.kV = newKv;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
  }

  public void updateMotionMagicConfigs() {
    double newExpoKa = intakeExpoKa.get();
    double newExpoKv = intakeExpoKv.get();

    if (newExpoKa != motionMagicConfigs.MotionMagicExpo_kA
        || newExpoKv != motionMagicConfigs.MotionMagicExpo_kV) {

      motionMagicConfigs.MotionMagicExpo_kA = newExpoKa;
      motionMagicConfigs.MotionMagicExpo_kV = newExpoKv;
      deployMotor.getConfigurator().apply(deployTalonFXConfigs);
    }
  }

  public void intakeLogs() {
    Logger.recordOutput("Mech/Intake/Current Deploy Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Intake/Desired Deploy Angle", desiredAngle.getDegrees());
    Logger.recordOutput("Mech/Intake/Desired Deploy Voltage", desiredDeployVoltage);
    Logger.recordOutput("Mech/Intake/Current Deploy Motor Output", deployMotor.get());
    Logger.recordOutput("Mech/Intake/Intake Hall Effect", isHallEffectTriggered());
    Logger.recordOutput("Mech/Intake/IsDeployed", isDeployed.getAsBoolean());
    Logger.recordOutput("Mech/Intake/Current Intake Motor Output", intakeMotor.get());
    Logger.recordOutput("Mech/Intake/Desired Intake Voltage", desiredIntakeVoltage);

    Logger.recordOutput(
        "Mech/Intake/ClosedLoopReference", deployMotor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput(
        "Mech/Intake/ClosedLoopError", deployMotor.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput(
        "Mech/Intake/Stator Current", deployMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Mech/Intake/Supply Current", deployMotor.getSupplyCurrent().getValueAsDouble());

    // SysID
    Logger.recordOutput("Mech/Intake/SysID/intakeSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      Logger.recordOutput(
          "Mech/Intake/SysID/intakeVoltage", deployMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Intake/SysID/intakePosition",
          getCurrentAngle().getRadians() / (2.0 * Math.PI)); // rotations
      Logger.recordOutput(
          "Mech/Intake/SysID/intakeVelocity", getVelocityRadPerSec() / (2.0 * Math.PI)); // rot/s
    }
  }
}
