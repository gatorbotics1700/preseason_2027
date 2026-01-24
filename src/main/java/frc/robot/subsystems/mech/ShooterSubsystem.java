package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class ShooterSubsystem extends SubsystemBase {
  public final TalonFX flywheelMotor;
  public final TalonFX kickerMotor;

  // private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public ShooterSubsystem() {
    flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    kickerMotor = new TalonFX(Constants.KICKER_MOTOR_HIGH_CAN_ID, TunerConstants.mechCANBus);
  }

  public void periodic() {
    SmartDashboard.putNumber("Shooter voltage", 15);
  }

  public void setFlywheelVoltage(double flywheelVoltage) {
    flywheelMotor.setVoltage(flywheelVoltage);
    System.out.println("SETTING FLYWHEEL VOLTAGE :)))");
  }

  public void setKickerVoltage(double kickerVoltage) {
    kickerMotor.setVoltage(kickerVoltage);
    System.out.println("SETTING KICKER VOLTAGE");
  }
}
