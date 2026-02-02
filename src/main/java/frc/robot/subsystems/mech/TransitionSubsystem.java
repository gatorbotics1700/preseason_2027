package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class TransitionSubsystem extends SubsystemBase {
  public final TalonFX kickerMotorHigh;
  public final TalonFX hopperMotor;

  public TransitionSubsystem() {
    kickerMotorHigh = new TalonFX(Constants.KICKER_MOTOR_HIGH_CAN_ID, TunerConstants.mechCANBus);
    hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  }

  public void periodic() {}

  public void setHighKickerVoltage(double kickerVoltage) {
    kickerMotorHigh.setVoltage(kickerVoltage);
    System.out.println("Setting kicker voltage!!!");
  }

  public void setHopperVoltage(double hopperVoltage) {
    hopperMotor.setVoltage(hopperVoltage);
  }
}
