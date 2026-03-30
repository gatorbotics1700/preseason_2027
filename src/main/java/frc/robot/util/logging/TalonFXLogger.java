package frc.robot.util.logging;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

/**
 * AdvantageKit logging for Phoenix 6 {@link TalonFX} telemetry.
 *
 * <p>Use a stable prefix per motor, e.g. {@code "Mech/Turret/motor"} or {@code
 * "Mech/Shooter/leftFlywheel"}.
 *
 * <p>Pass {@code sysIdExtras == true} only while SysId routines are running (or from a config flag
 * when a mechanism is still being tuned). Mechanism-specific SysId quantities (e.g. turret angle in
 * mechanism units) should still be logged next to this from the subsystem.
 */
public final class TalonFXLogger {

  private TalonFXLogger() {}

  public static void log(TalonFX motor, String category, String mechanismName) {
    log(motor, category, mechanismName, "");
  }

  public static void log(TalonFX motor, String category, String mechanismName, String motorName) {
    String prefix = category + "/" + mechanismName + (motorName.equals("") ? "" : "/" + motorName);

    Logger.recordOutput(prefix + "/Motor Output", motor.get());
    // NOTE: For SysID, velocity needs to be in proper units that rely on gear ratios, so will need
    // to log in the subsystem as well
    Logger.recordOutput(prefix + "/Motor Velocity", motor.getVelocity().getValueAsDouble());
    Logger.recordOutput(prefix + "/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(prefix + "/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        category + "/All Stator Currents/" + (motorName.equals("") ? mechanismName : motorName),
        motor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(prefix + "/MotorVoltage", motor.getMotorVoltage().getValueAsDouble());
    // NOTE: position needs to be in mechanism units not motor units, so log in subsystem as well
    Logger.recordOutput(prefix + "/Motor Position", motor.getPosition().getValueAsDouble());
    Logger.recordOutput(prefix + "/SupplyVoltage", motor.getSupplyVoltage().getValueAsDouble());
    Logger.recordOutput(prefix + "/DeviceTemp", motor.getDeviceTemp().getValueAsDouble());

    Logger.recordOutput(
        prefix + "/ClosedLoop/ClosedLoopReference",
        motor.getClosedLoopReference().getValueAsDouble());
    Logger.recordOutput(
        prefix + "/ClosedLoop/ClosedLoopError", motor.getClosedLoopError().getValueAsDouble());
    Logger.recordOutput(
        prefix + "/ClosedLoop/ClosedLoopFeedForward",
        motor.getClosedLoopFeedForward().getValueAsDouble());
  }
}
