package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// this is what we will use in constant tracking command!

// @AutoLog
public class ShotCalculator {

  public ShotCalculator() {}

  public static ShotParameters calculateShot(
      Pose2d drivetrainPose, ChassisSpeeds drivetrainVelocity, Translation3d target) {
    Rotation2d fieldRelativeTurretAngle = new Rotation2d(0); // TODO add real math for this :)
    Rotation2d turretAngle =
        fieldRelativeTurretAngle.minus(drivetrainPose.getRotation()); // TODO check the math on this
    Rotation2d hoodAngle = new Rotation2d(0); // TODO add real math for this :)
    return new ShotParameters(turretAngle, hoodAngle);
  }
}
