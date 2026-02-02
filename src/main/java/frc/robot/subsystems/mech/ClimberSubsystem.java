package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ClimberSubsystem extends SubsystemBase {

  public final TalonFX motor;

  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final PIDController pidController;
  private double output;

  private static final double kP = 0.0; // TODO: tune all of these
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private final double DEADBAND = inchesToRevs(1);

  public ClimberSubsystem() {
    motor = new TalonFX(Constants.OUTER_ARM_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    motor.setNeutralMode(NeutralModeValue.Brake);
    pidController = new PIDController(kP, kI, kD);
  }

  public void setMotorOutput(double speed) {
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.setControl(dutyCycleOut.withOutput(speed));
  }

  public void moveArm(double desiredRevs) {
    motor.setNeutralMode(NeutralModeValue.Brake);
    double error = desiredRevs - getCurrentMotorRevs();
    if (Math.abs(error) > DEADBAND) {
      output = pidController.calculate(error);
    } else {
      output = 0;
    }
    setMotorOutput(0);
  }

  public double getCurrentMotorRevs() {
    return motor.getPosition().getValueAsDouble();
  }

  public double inchesToRevs(double desiredInches) {
    return desiredInches * Constants.CLIMBER_TICKS_PER_INCH;
  }

  public double getMotorOutput() {
    return output;
  }
}
