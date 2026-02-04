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

import edu.wpi.first.wpilibj.Timer;

/** Simulated hood IO: integrates speed to get position in revs. */
public class HoodIOSim implements HoodIO {
  private static final double MAX_REVS_PER_SEC = 0.5; // rev/s at full duty cycle

  private double positionRevs = 0.0;
  private double speed = 0.0;
  private double lastTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    positionRevs += speed * MAX_REVS_PER_SEC * dt;

    inputs.positionRevs = positionRevs;
    inputs.velocityRevsPerSec = speed * MAX_REVS_PER_SEC;
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = Math.max(-1.0, Math.min(1.0, speed));
  }
}
