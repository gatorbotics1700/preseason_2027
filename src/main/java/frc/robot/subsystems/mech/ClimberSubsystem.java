package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ClimberSubsystem extends SubsystemBase {

  private static final int CLIMBER_GEAR_RATIO = 81; // TODO get a real number
  private static final double WINCH_INCHES_PER_REV = 1; // TODO get a real number
  public final TalonFX motor;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private double desiredPositionInches;
  // motion magic stuff goes here
  private final double DEADBAND_INCHES = 1;

  public ClimberSubsystem() {
    motor = new TalonFX(Constants.CLIMBER_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    motor.setNeutralMode(NeutralModeValue.Brake);
    // motion magic stuff
  }

  public void periodic() {
    motor.setNeutralMode(NeutralModeValue.Brake); // do we really need this here?
    double error = desiredPositionInches - currentPositionInches();
    if (Math.abs(error) > DEADBAND_INCHES) {
      setMotorOutput(0.2 * error);
    } else {
      setMotorOutput(0);
    }
  }

  private void setMotorOutput(double speed) {
    motor.setNeutralMode(NeutralModeValue.Brake); // do we really need this here?
    motor.setControl(dutyCycleOut.withOutput(speed));
  }

  public double currentPositionInches() {
    return motor.getPosition().getValueAsDouble()
        / Constants.KRAKEN_TICKS_PER_REV
        * CLIMBER_GEAR_RATIO
        * WINCH_INCHES_PER_REV; // TODO majorly check this math
  }

  public void setDesiredPositionInches(double desiredPositionInches) {
    this.desiredPositionInches = desiredPositionInches;
  }
}
