package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

// @AutoLog
public class ShotParameters {
  public Rotation2d turretAngle;
  public Rotation2d hoodAngle;

  public ShotParameters(Rotation2d turretAngle, Rotation2d hoodAngle) {
    this.turretAngle = turretAngle;
    this.hoodAngle = hoodAngle;
  }
}
