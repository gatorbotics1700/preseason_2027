package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class IntakeSubsystem extends SubsystemBase {

  public final TalonFX intakeMotor;
  public final TalonFX extensionMotor;

  private PIDController pidController;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private double positionError;
  private double extensionVoltage;

  private final double kP = 0.0; // TODO: tune all of these
  private final double kI = 0.0;
  private final double kD = 0.0;

  public final double EXTENDED_POSITION = degreesToTicks(90); // ticks, TODO: change
  public final double RETRACTED_POSITION = degreesToTicks(0); // ticks, TODO: change
  public final double DEADBAND = 2;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    extensionMotor = new TalonFX(Constants.EXTENSION_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    pidController = new PIDController(kP, kI, kD);
  }

  @Override
  public void periodic() {}

  public void setIntakeVoltage(double intakeVoltage) {
    intakeMotor.setVoltage(intakeVoltage);
  }

  public void setExtensionVoltage(double output) {
    extensionMotor.setControl(dutyCycleOut.withOutput(output));
  }

  public double getPosition() {
    return extensionMotor.getPosition().getValueAsDouble();
  }

  public void pivotIntake(boolean wantExtended) {
    if (wantExtended) {
      positionError = EXTENDED_POSITION - getPosition();
    } else {
      positionError = RETRACTED_POSITION - getPosition();
    }
    if (Math.abs(positionError) > DEADBAND) {
      extensionVoltage = pidController.calculate(positionError);
      setExtensionVoltage(extensionVoltage);
    } else {
      setExtensionVoltage(0);
    }
  }

  public double degreesToTicks(double degrees) {
    return degrees * Constants.HOOD_GEAR_RATIO / 360; // TODO: fix this when we know mech details
  }
}
