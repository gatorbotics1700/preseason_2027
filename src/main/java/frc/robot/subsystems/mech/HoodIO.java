// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.mech;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double angleDeg = 0.0;
    public double velocityDegPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified angle. */
  public default void setAngle(double angleDeg) {}

  /** Stop the hood. */
  public default void stop() {}

  /** Set the brake mode. */
  public default void setBrakeMode(boolean enable) {}
}