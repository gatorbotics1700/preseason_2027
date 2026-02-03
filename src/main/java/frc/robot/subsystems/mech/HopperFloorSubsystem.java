package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class HopperFloorSubsystem extends SubsystemBase {
  public static final double HOPPER_FLOOR_SPEED = 0;
  public final TalonFX hopperMotor;
  private double hopperSpeed;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  // motion magic stuff

  public HopperFloorSubsystem() {
    hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  }

  public void periodic() {
    hopperMotor.setControl(dutyCycleOut.withOutput(hopperSpeed)); // TODO replace with motion magic
  }

  public void setHopperFloorSpeed(double hopperSpeed) {
    this.hopperSpeed = hopperSpeed;
  }
}
