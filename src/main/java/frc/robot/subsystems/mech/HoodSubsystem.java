package frc.robot.subsystems.mech;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HoodSubsystem extends SubsystemBase {

  private final DigitalInput limitSwitch;

  // when false, run voltage control
  private boolean positionControl = true;

  private static final double SYSID_LIMIT_MARGIN_DEGREES = 2;

  private final TalonFX hoodMotor =
      new TalonFX(HoodConstants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  private TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private boolean sysIdRunning = false;
  private SysIdRoutine sysIdRoutine;
  private final VoltageOut sysIdVoltageRequest = new VoltageOut(0);

  public static final LoggedNetworkNumber desiredHoodAngle =
      new LoggedNetworkNumber("/Tuning/Hood/Angle", 68);
  private Rotation2d desiredAngle = getCurrentAngle(); // HoodConstants.RETRACTED_POSITION;

  // Tunable PID gains for hood control
  public static final LoggedNetworkNumber hoodKp = new LoggedNetworkNumber("/Tuning/Hood/kP", 4.8);
  public static final LoggedNetworkNumber hoodKi = new LoggedNetworkNumber("/Tuning/Hood/kI", 0.0);
  public static final LoggedNetworkNumber hoodKd = new LoggedNetworkNumber("/Tuning/Hood/kD", 0.1);

  public HoodSubsystem() {
    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    // if (RobotConfigLoader.getSerialNumber().equals(RobotConfigLoader.NILE_SERIAL)) {
    //   talonFXConfigs.withMotorOutput(
    //       new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    // } else {
    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
    // }

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kG =
        0.2; // Add 0.2128 V output to overcome gravity (tuned in early feedforward testing)
    slot0Configs.kS =
        0.25; // Add 0.01 V output to overcome static friction (just a guesstimate, but this might
    // just be 0
    slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    // Initial PID gains come from tunable LoggedNetworkNumbers
    slot0Configs.kP = hoodKp.get(); // A position error of 2.5 rotations results in 12V output
    slot0Configs.kI = hoodKi.get(); // no output for integrated error
    slot0Configs.kD = hoodKd.get(); // a velocity error of 1 rps results in 0.1 V output

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
    // Update PID gains from NetworkTables if they've changed, and reapply configs
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    double newKp = hoodKp.get();
    double newKi = hoodKi.get();
    double newKd = hoodKd.get();
    if (newKp != slot0Configs.kP || newKi != slot0Configs.kI || newKd != slot0Configs.kD) {
      slot0Configs.kP = newKp;
      slot0Configs.kI = newKi;
      slot0Configs.kD = newKd;
      hoodMotor.getConfigurator().apply(talonFXConfigs);
    }

    // Skip limit switch safety during SysID - the isSysIdOutOfBounds() handles limits
    if (limitSwitch.get() && !sysIdRunning) {
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

    // Only run position control if SysID is not running
    if (positionControl && !sysIdRunning) {
      setHoodPosition(desiredAngle);
    }

    hoodLogs();
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

  private double getVelocityRadPerSec() {
    double motorRPS = hoodMotor.getVelocity().getValueAsDouble();
    return motorRPS
        / HoodConstants.HOOD_GEAR_RATIO
        / HoodConstants.HOOD_SHAFT_REVS_PER_MECH_REV
        * 2
        * Math.PI;
  }

  public Rotation2d convertLaunchAngleToHoodAngle(Rotation2d launchAngle) {
    return launchAngle; // TODO: fix this to actually be right
  }

  private void initSysIdRoutine() {
    // config for our test. Sets voltage ramps, limits, and a logging callback
    SysIdRoutine.Config config =
        new SysIdRoutine.Config(
            // this is the ramp rate for voltage during a test
            Volts.per(Second).of(1),
            // this is the maximum voltage for the test
            Volts.of(4),
            // this is the duration of the test.
            Seconds.of(1.5),
            (state) -> Logger.recordOutput("Mech/Hood/SysID/SysIdState", state.toString()));

    // mechanism for our test. Sets the voltage; we log voltage/position/velocity ourselves in
    // periodic()
    SysIdRoutine.Mechanism mechanism =
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              sysIdRunning = true;
              hoodMotor.setControl(sysIdVoltageRequest.withOutput(voltage.in(Volts)));
            },
            null, // Log via AdvantageKit in periodic() so data goes to the same log file
            this,
            "hood");
    System.out.println("CREATING NEW SYSID ROUTINE");
    sysIdRoutine = new SysIdRoutine(config, mechanism);
  }

  private boolean isSysIdOutOfBounds() {
    double angleDeg = getCurrentAngle().getDegrees();
    Logger.recordOutput("Mech/Hood/SysID/SysID Current Angle", angleDeg);
    boolean outOfBounds =
        angleDeg > HoodConstants.RETRACTED_POSITION.getDegrees() + SYSID_LIMIT_MARGIN_DEGREES
            || angleDeg < HoodConstants.MIN_ANGLE.getDegrees() - SYSID_LIMIT_MARGIN_DEGREES;
    Logger.recordOutput("Mech/Hood/SysID/SysId Out Of Bounds", outOfBounds);
    Logger.recordOutput(
        "Mech/Hood/SysID/SysId Retracted Limit",
        HoodConstants.RETRACTED_POSITION.getDegrees() + SYSID_LIMIT_MARGIN_DEGREES);
    Logger.recordOutput(
        "Mech/Hood/SysID/SysId Min Limit",
        HoodConstants.MIN_ANGLE.getDegrees() - SYSID_LIMIT_MARGIN_DEGREES);
    return outOfBounds;
  }

  // run under a series of "flat" voltages to measure velocity behavior
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    System.out.println("RUNNING SYSID QUASISTATIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .quasistatic(direction)
        .until(this::isSysIdOutOfBounds) // temporarily disabled for testing
        .finallyDo(
            () -> {
              setHoodVoltage(0);
              sysIdRunning = false;
            })
        .withName("Hood SysId Quasistatic " + direction);
  }

  // measure acceleration behavior
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    System.out.println("RUNNING SYSID DYNAMIC");
    if (sysIdRoutine == null) {
      initSysIdRoutine();
    }
    return sysIdRoutine
        .dynamic(direction)
        .until(this::isSysIdOutOfBounds) // temporarily disabled for testing
        .finallyDo(
            () -> {
              setHoodVoltage(0);
              sysIdRunning = false;
            })
        .withName("Hood SysId Dynamic " + direction);
  }

  public void hoodLogs() {
    Logger.recordOutput("Mech/Hood/Desired Angle", desiredAngle.getDegrees());
    Logger.recordOutput("Mech/Hood/Current Angle", getCurrentAngle().getDegrees());
    Logger.recordOutput("Mech/Hood/Motor Output", hoodMotor.get());
    Logger.recordOutput("Mech/Hood/Current velocity", hoodMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Mech/Hood/Limit switch", isRetractedLimitSwitchPressed());
    Logger.recordOutput(
        "Mech/Hood/Control Mode", positionControl ? "position control" : "voltage control");

    // SysID
    Logger.recordOutput("Mech/Hood/SysID/hoodSysIDRunning", sysIdRunning);
    if (sysIdRunning) {
      Logger.recordOutput(
          "Mech/Hood/SysID/hoodVoltage", hoodMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput(
          "Mech/Hood/SysID/hoodPosition",
          getCurrentAngle().getRadians() / (2.0 * Math.PI)); // rotations
      Logger.recordOutput(
          "Mech/Hood/SysID/hoodVelocity", getVelocityRadPerSec() / (2.0 * Math.PI)); // rot/s
    }
  }
}
