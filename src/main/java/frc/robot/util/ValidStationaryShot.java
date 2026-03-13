package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ValidStationaryShot {
  public Pose2d pose; // robot relative
  public Rotation2d hoodAngle; // degrees from vertical
  public double shotSpeed; // mps of the ball's exit velo

  // (note: shotSpeed is NOT the same as the kraken's speed)

  public ValidStationaryShot(Pose2d pose, Rotation2d hoodAngle, double shotSpeed) {
    this.pose = pose;
    this.hoodAngle = hoodAngle;
    this.shotSpeed = shotSpeed;
  }
}
