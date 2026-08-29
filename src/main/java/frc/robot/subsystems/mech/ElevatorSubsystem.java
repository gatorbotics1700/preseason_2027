package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(0, TunerConstants.mechCANBus);
  private TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;
  // Tunable PID gains for elevator control
  public static final LoggedNetworkNumber elevatorKp =
      new LoggedNetworkNumber("/Tuning/Elevator/kP", 30);
  public static final LoggedNetworkNumber elevatorKi =
      new LoggedNetworkNumber("/Tuning/Elevator/kI", 0.0);
  public static final LoggedNetworkNumber elevatorKd =
      new LoggedNetworkNumber("/Tuning/Elevator/kD", 0.1);

  public static final LoggedNetworkNumber elevatorKg =
      new LoggedNetworkNumber("/Tuning/Elevator/kG", 0.2);
  public static final LoggedNetworkNumber elevatorKs =
      new LoggedNetworkNumber("/Tuning/Elevator/kS", 0.01);
  public static final LoggedNetworkNumber elevatorKv =
      new LoggedNetworkNumber("/Tuning/Elevator/kV", 0.16);
  public static final LoggedNetworkNumber elevatorKa =
      new LoggedNetworkNumber("/Tuning/Elevator/kA", 0.01);

  public ElevatorSubsystem() {
    talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = elevatorKg.get(); // Add ____V output to overcome gravity
    slot0Configs.kS = elevatorKs.get(); // Add _____ V output to overcome static friction

    slot0Configs.kV = elevatorKv.get(); // Output per unit of target velocity (output/rps)
    slot0Configs.kA = elevatorKa.get(); // Output per unit of target (output/(rps/s))

    // Initial PID gains come from tunable LoggedNetworkNumbers
    slot0Configs.kP = elevatorKp.get(); // Output per unit of error in position (output/rotation)
    slot0Configs.kI =
        elevatorKi.get(); // Output per unit of integrated error in position (output/(rotation*s))
    slot0Configs.kD = elevatorKd.get();

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // when 0, unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV =
        0.16; // Voltage required to apply a given acceleration (V/(rps/s))
    motionMagicConfigs.MotionMagicExpo_kA =
        0.2; // Voltage required to maintain a given velocity (V/rps)

    elevatorMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void periodic() {
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    double newKp = elevatorKp.get();
    double newKi = elevatorKi.get();
    double newKd = elevatorKd.get();

    double newKg = elevatorKg.get();
    double newKs = elevatorKs.get();
    double newKv = elevatorKv.get();
    double newKa = elevatorKa.get();
    if (newKp != slot0Configs.kP
        || newKi != slot0Configs.kI
        || newKd != slot0Configs.kD
        || newKg != slot0Configs.kG
        || newKs != slot0Configs.kS
        || newKv != slot0Configs.kV
        || newKa != slot0Configs.kA) {
      slot0Configs.kP = newKp;
      slot0Configs.kI = newKi;
      slot0Configs.kD = newKd;
      slot0Configs.kG = newKg;
      slot0Configs.kS = newKs;
      slot0Configs.kV = newKv;
      slot0Configs.kA = newKa;

      elevatorMotor.getConfigurator().apply(talonFXConfigs);
    }
  }

  public void setDesiredPosition(double rotations) {
    double desiredRotations = rotations;
    elevatorMotor.setControl(m_request.withPosition(desiredRotations));
  }

  public double getCurrentPosition() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean atDesiredPosition(double desiredPosition, double error) {
    if (Math.abs(getCurrentPosition() - desiredPosition) <= error) {
      return true;
    }
    return false;
  }
}
