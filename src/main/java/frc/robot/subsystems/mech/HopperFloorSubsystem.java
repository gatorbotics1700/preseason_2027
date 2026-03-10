package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class HopperFloorSubsystem extends SubsystemBase {
  private final TalonFX hopperMotor;
  private double desiredHopperVoltage;
  private static TalonFXConfiguration talonFXConfigs;
  private static Slot0Configs slot0Configs;
  private static MotionMagicVelocityVoltage m_velocity;
  public static LoggedNetworkNumber hopperVoltage =
      new LoggedNetworkNumber("/Tuning/hopperVoltage", 8);

  public HopperFloorSubsystem() {
    hopperMotor = new TalonFX(HopperFloorConstants.HOPPER_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    // TALONFX CONFIGS & MOTION MAGIC VELOCITY VOLTAGE CONTROL // TODO check if this works with
    // motionMagicVelocityVoltage - may want to delete some values
    desiredHopperVoltage = 0.0;
    m_velocity = new MotionMagicVelocityVoltage(0);
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12-0.2 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 0.11; // A position error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // no output for error derivative

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    hopperMotor.getConfigurator().apply(talonFXConfigs);

    m_velocity.Slot = 0; // TODO check what this does
  }

  public void periodic() {
    Logger.recordOutput("Mech/Hopper Floor/Desired Velocity", desiredHopperVoltage);
    Logger.recordOutput("Mech/Hopper Floor/Motor Output", hopperMotor.get());
    hopperMotor.setVoltage(desiredHopperVoltage);
  }

  public void setDesiredHopperFloorVoltage(double voltage) {
    // if (voltage == 0.0) {
    //   this.desiredHopperVoltage = 0.0;
    // } else {
    //   this.desiredHopperVoltage = hopperVoltage.get();
    // }
    this.desiredHopperVoltage = voltage;
  }
}
