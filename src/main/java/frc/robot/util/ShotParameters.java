package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// @AutoLog
public class ShotParameters {
  public Rotation2d turretAngle; // robot relative
  public Rotation2d hoodAngle; // degrees from vertical
  public double shotSpeed; // mps of the ball's exit velo
  public Pose2d pose; // robot pose field relative

  // (note: shotSpeed is NOT the same as the kraken's speed)

  public ShotParameters(Rotation2d turretAngle, Rotation2d hoodAngle, double shotSpeed) {
    this.turretAngle = turretAngle;
    this.hoodAngle = hoodAngle;
    this.shotSpeed = shotSpeed;
  }

  public ShotParameters(Pose2d
   pose, Rotation2d hoodAngle, double shotSpeed) {
    this.pose = pose;
    this.hoodAngle = hoodAngle;
    this.shotSpeed = shotSpeed;
  }
}
