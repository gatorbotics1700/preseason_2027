package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {
  public static final double TRANSITION_SPEED = 0;
  private final TalonFX flywheelMotor;
  private final TalonFX transitionMotor;
  private double desiredFlywheelSpeed;
  private double transitionSpeed;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  // motion magic stuff

  // private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public ShooterSubsystem() {
    flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    transitionMotor = new TalonFX(31, TunerConstants.mechCANBus);
    // motion magic stuff
  }

  public void periodic() {
    flywheelMotor.setControl(
        dutyCycleOut.withOutput(desiredFlywheelSpeed)); // TODO use motion magic to keep at speed
    transitionMotor.setControl(
        dutyCycleOut.withOutput(
            transitionSpeed)); // TODO double check that this actually sets speed in the way we
    // think it does
  }

  public void setFlywheelSpeed(double desiredFlywheelSpeed) {
    this.desiredFlywheelSpeed = desiredFlywheelSpeed;
  }

  public double getFlywheelSpeed() {
    return flywheelMotor
        .getRotorVelocity()
        .getValueAsDouble(); // TODO figure out if this is the right method
  }

  public void setTransitionSpeed(double transitionSpeed) {
    this.transitionSpeed = transitionSpeed;
  }
}
