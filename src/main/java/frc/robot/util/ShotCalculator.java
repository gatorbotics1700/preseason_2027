package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

// this is what we will use in constant tracking command!

// @AutoLog
public class ShotCalculator {

  public ShotCalculator() {}

  public static ShotParameters calculateShot(
      Pose2d drivetrainPose, ChassisSpeeds drivetrainVelocity, Translation3d target) {
    // calculate field relative shooter pose
    Translation2d botToShooter =
        new Translation2d(
            Constants.BOT_RELATIVE_SHOOTER_POSE.getX(), Constants.BOT_RELATIVE_SHOOTER_POSE.getY());
    Transform2d botToShooterTransform = new Transform2d(botToShooter, new Rotation2d(0));
    Pose2d shooterPose = drivetrainPose.transformBy(botToShooterTransform);

    // calculate turret angle
    double deltaY = target.getY() - shooterPose.getY();
    double deltaX = target.getX() - shooterPose.getX();
    Rotation2d fieldRelativeTurretAngle =
        new Rotation2d(Math.atan2(deltaY, deltaX)); // TODO add real math for this :)
    Rotation2d turretAngle =
        fieldRelativeTurretAngle.minus(drivetrainPose.getRotation()); // TODO check the math on this

    // calculate hood angle
    Pose3d shooterPose3d =
        new Pose3d(
            shooterPose.getX(),
            shooterPose.getY(),
            Constants.BOT_RELATIVE_SHOOTER_POSE.getZ(),
            new Rotation3d(0, 0, turretAngle.getRadians()));

    Rotation2d hoodAngle = new Rotation2d(0); // TODO add real math for this :)
    return new ShotParameters(turretAngle, hoodAngle);
  }
}
