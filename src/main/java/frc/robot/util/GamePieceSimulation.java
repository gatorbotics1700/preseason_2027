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

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Manages game piece simulation for shooting in maple-sim.
 *
 * <p>Simulates fuel ball trajectories with realistic physics including: - Gravity - Air drag -
 * Initial velocity from shooter + robot motion - Visualization in AdvantageScope
 */
public class GamePieceSimulation {
  /** Represents a single fuel ball in flight. */
  private static class FuelBall {
    public Translation3d position;
    public Translation3d velocity; // m/s in x, y, z
    public double birthTime; // When the ball was launched
    public final double launchTime;

    public FuelBall(Translation3d position, Translation3d velocity, double launchTime) {
      this.position = position;
      this.velocity = velocity;
      this.birthTime = Timer.getFPGATimestamp();
      this.launchTime = launchTime;
    }

    /** Returns true if ball has hit the ground. */
    public boolean isOnGround() {
      return position.getZ() <= 0.0;
    }

    /** Returns age of ball in seconds. */
    public double getAge() {
      return Timer.getFPGATimestamp() - birthTime;
    }
  }

  private final List<FuelBall> activeBalls = new ArrayList<>();
  private static final double MAX_BALL_AGE = 15.0; // Remove balls after 15 seconds
  private static final double DT = 0.02; // 20ms update rate
  private static final double AIR_DENSITY_KG_M3 = 1.2754;
  private static final double FUEL_DRAG_COEFFICIENT = 0;
  private static final double GRAVITY_MPS2 = 9.8;
  private static final double FUEL_CROSS_SECTION_AREA = 0.0176;
  private static final double FUEL_MASS_KG = 0.215;

  /**
   * Launch a fuel ball from the shooter.
   *
   * @param shooterPosition 3D position of shooter exit on field
   * @param shooterVelocity Velocity of game piece from shooter (field frame)
   * @param launchAngle Hood angle (from horizontal)
   * @param turretAngle Turret angle (field-relative)
   */
  public void launchFuelBall(
      Translation3d shooterPosition,
      double exitVelocityMps,
      Rotation2d launchAngle,
      Rotation2d turretAngle) {

    if (Constants.currentMode != Constants.Mode.SIM) {
      return; // Only simulate in sim mode
    }

    System.out.println("LAUNCHING BALL AT " + exitVelocityMps + " MPS");

    // Calculate initial velocity vector in field frame
    double vx = exitVelocityMps * Math.cos(launchAngle.getRadians()) * turretAngle.getCos();
    double vy = exitVelocityMps * Math.cos(launchAngle.getRadians()) * turretAngle.getSin();
    double vz = exitVelocityMps * Math.sin(launchAngle.getRadians());

    Translation3d initialVelocity = new Translation3d(vx, vy, vz);

    FuelBall ball = new FuelBall(shooterPosition, initialVelocity, Timer.getFPGATimestamp());
    activeBalls.add(ball);

    Logger.recordOutput("GamePiece/LaunchedBall", new Pose3d(shooterPosition, new Rotation3d()));
    Logger.recordOutput("GamePiece/LaunchVelocity", initialVelocity.getNorm());
  }

  /**
   * Update all active fuel balls (call this periodically).
   *
   * <p>Simulates physics and removes balls that have landed or aged out.
   */
  public void updateBalls() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }

    List<FuelBall> ballsToRemove = new ArrayList<>();

    for (FuelBall ball : activeBalls) {
      // Update physics
      updateBallPhysics(ball, DT);

      // Check if ball should be removed (only remove if too old, keep grounded balls visible)
      if (ball.getAge() > MAX_BALL_AGE) {
        ballsToRemove.add(ball);
      }
    }

    // Remove old balls
    activeBalls.removeAll(ballsToRemove);

    // Log all active ball positions
    Pose3d[] ballPoses = new Pose3d[activeBalls.size()];
    for (int i = 0; i < activeBalls.size(); i++) {
      ballPoses[i] = new Pose3d(activeBalls.get(i).position, new Rotation3d());
    }
    Logger.recordOutput("GamePiece/ActiveBalls", ballPoses);
    Logger.recordOutput("GamePiece/ActiveBallCount", activeBalls.size());
  }

  /**
   * Update a single ball's physics for one timestep.
   *
   * @param ball The ball to update
   * @param dt Time step in seconds
   */
  private void updateBallPhysics(FuelBall ball, double dt) {
    // Calculate drag force
    double speed = ball.velocity.getNorm();

    // Avoid division by zero
    if (speed < 0.001) {
      return; // Ball is essentially stopped
    }

    double dragForce =
        0.5 * AIR_DENSITY_KG_M3 * FUEL_DRAG_COEFFICIENT * FUEL_CROSS_SECTION_AREA * speed * speed;

    // Drag acceleration = -drag_force / mass * velocity_unit_vector
    // velocity_unit_vector = velocity / speed
    double dragAccelMagnitude = dragForce / FUEL_MASS_KG;
    Translation3d dragAccel = ball.velocity.times(-dragAccelMagnitude / speed);

    // Gravity acceleration
    Translation3d gravityAccel = new Translation3d(0, 0, -GRAVITY_MPS2);

    // Total acceleration
    Translation3d totalAccel = dragAccel.plus(gravityAccel);

    // Update velocity (semi-implicit Euler)
    ball.velocity = ball.velocity.plus(totalAccel.times(dt));

    // Update position
    ball.position = ball.position.plus(ball.velocity.times(dt));

    // Clamp to ground
    if (ball.position.getZ() < 0) {
      ball.position = new Translation3d(ball.position.getX(), ball.position.getY(), 0);
      ball.velocity = new Translation3d(0, 0, 0); // Stop on ground
    }
  }

  /** Clear all active balls. */
  public void clearBalls() {
    activeBalls.clear();
  }

  /** Returns the number of active balls. */
  public int getActiveBallCount() {
    return activeBalls.size();
  }

  /**
   * Get the closest ball to a target position (for hit detection).
   *
   * @param target Target position
   * @param maxDistance Maximum distance to consider a "hit"
   * @return Distance to closest ball, or -1 if no balls within maxDistance
   */
  public double getClosestBallDistance(Translation3d target, double maxDistance) {
    double closestDistance = Double.MAX_VALUE;

    for (FuelBall ball : activeBalls) {
      double distance = ball.position.getDistance(target);
      if (distance < closestDistance) {
        closestDistance = distance;
      }
    }

    return closestDistance < maxDistance ? closestDistance : -1.0;
  }

  /**
   * Check if any ball has passed through a target zone recently.
   *
   * @param target Target position
   * @param radius Radius of target zone
   * @param checkAgeSec Only check balls launched within this many seconds
   * @return True if a ball passed through the target
   */
  public boolean checkTargetHit(Translation3d target, double radius, double checkAgeSec) {
    for (FuelBall ball : activeBalls) {
      if (ball.getAge() <= checkAgeSec) {
        double distance = ball.position.getDistance(target);
        if (distance <= radius) {
          return true;
        }
      }
    }
    return false;
  }
}
