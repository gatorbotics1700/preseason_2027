package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {
  public final TalonFX flywheelMotor;
  public double flywheelVoltagePrint; // this value is solely to print the voltage to elastic
  public final TalonFX kickerMotor;

  // private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public ShooterSubsystem() {
    flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    kickerMotor = new TalonFX(31, TunerConstants.mechCANBus);
    flywheelVoltagePrint = 0.0;
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter voltage", flywheelVoltagePrint);
  }

  public void setFlywheelVoltage(double flywheelVoltage) {
    flywheelVoltagePrint = flywheelVoltage;
    flywheelMotor.setVoltage(flywheelVoltage);
    System.out.println("SETTING FLYWHEEL VOLTAGE :)))");
  }
}
