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
    /** Motor position in revolutions. */
    public double positionRevs = 0.0;
    /** Motor velocity in revolutions per second. */
    public double velocityRevsPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Run open loop at the specified speed (duty cycle, typically -1 to 1). */
  public default void setSpeed(double speed) {}
}
