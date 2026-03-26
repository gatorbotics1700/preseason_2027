package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class HopperFloorSubsystem extends SubsystemBase {
  private final TalonFX hopperMotor;
  private double desiredHopperSpeed;

  public HopperFloorSubsystem() {
    hopperMotor = new TalonFX(HopperFloorConstants.HOPPER_MOTOR_CAN_ID, TunerConstants.mechCANBus);

    desiredHopperSpeed = 0.0;
  }

  public void periodic() {
    hopperMotor.set(desiredHopperSpeed);
    hopperFloorLogs();
  }

  public void setDesiredHopperFloorSpeed(double speed) {
    this.desiredHopperSpeed = speed;
  }

  public void hopperFloorLogs() {
    Logger.recordOutput("Mech/Hopper Floor/Desired Speed", desiredHopperSpeed);
    Logger.recordOutput("Mech/Hopper Floor/Motor Output", hopperMotor.get());
  }
}
