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

// NOT COMPLETE
package frc.robot.subsystems.mech;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.mech.ShooterIO.ShooterIOInputs;

/** Simulated hood IO: integrates speed to get position in revs. */
public class ShooterIOSim implements ShooterIO {

  private double positionRevs = 0.0;
  private double flywheelSpeed = 0.0;
  private double transitionSpeed = 0.0;
  private double lastTime = Timer.getFPGATimestamp();

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;

    inputs.flywheelVelocityRevsPerSec = flywheelSpeed;
    inputs.transitionVelocityRevsPerSec = transitionSpeed;
  }

  @Override
  public void setSpeed(double speed) {
    this.flywheelSpeed = speed;
    this.transitionSpeed = speed;
  }
}
