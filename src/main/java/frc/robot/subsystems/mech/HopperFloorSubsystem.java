package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class HopperFloorSubsystem extends SubsystemBase {
  private final TalonFX hopperMotor;
  private double desiredHopperVoltage;

  public HopperFloorSubsystem() {
    hopperMotor = new TalonFX(HopperFloorConstants.HOPPER_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    // TALONFX CONFIGS & MOTION MAGIC VELOCITY VOLTAGE CONTROL // TODO check if this works with
    // motionMagicVelocityVoltage - may want to delete some values
    desiredHopperVoltage = 0.0;
  }

  public void periodic() {
    hopperMotor.setVoltage(desiredHopperVoltage);
    hopperFloorLogs();
  }

  public void setDesiredHopperFloorVoltage(double voltage) {
    this.desiredHopperVoltage = voltage;
  }

  public void hopperFloorLogs() {
    Logger.recordOutput("Mech/Hopper Floor/Desired Voltage", desiredHopperVoltage);
    Logger.recordOutput("Mech/Hopper Floor/Motor Output", hopperMotor.get());
  }
}
