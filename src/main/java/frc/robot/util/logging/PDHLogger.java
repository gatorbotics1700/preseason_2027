package frc.robot.util.logging;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.littletonrobotics.junction.Logger;

/** AdvantageKit logging for REV PDH / CTRE PDP per-channel current and bus voltage. */
public final class PDHLogger {

  private PDHLogger() {}

  /**
   * Logs all PD channels (amps), total current, and input voltage. No-op if {@code pdh} is null
   * (e.g. log replay mode).
   */
  public static void log(PowerDistribution pdh) {
    if (pdh == null) {
      return;
    }
    int channels = pdh.getNumChannels();
    for (int ch = 0;
        ch < channels;
        ch++) { // TODO: if this does the same thing as getAllCurrents, get rid of it
      Logger.recordOutput("PDH/Channel " + ch + " Current", pdh.getCurrent(ch));
    }
    Logger.recordOutput("PDH/TotalCurrent", pdh.getTotalCurrent());
    Logger.recordOutput("PDH/Voltage", pdh.getVoltage());
    Logger.recordOutput("PDH/Total Power", pdh.getTotalPower());
    Logger.recordOutput("PDH/Total Energy", pdh.getTotalEnergy());
    Logger.recordOutput("PDH/Temperature", pdh.getTemperature());
    Logger.recordOutput("PDH/All Currents", pdh.getAllCurrents());
  }
}
