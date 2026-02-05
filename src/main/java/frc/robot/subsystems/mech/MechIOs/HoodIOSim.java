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

package frc.robot.subsystems.mech.MechIOs;

import edu.wpi.first.wpilibj.Timer;

/** Simulated hood IO: integrates speed to get position in revs. */
public class HoodIOSim implements HoodIO {

  private double positionRevs = 0.0;
  private double speed = 0.0;
  private double lastTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    positionRevs += speed * dt;

    inputs.positionRevs = positionRevs;
    inputs.velocityRevsPerSec = speed;
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = speed;
  }
}
