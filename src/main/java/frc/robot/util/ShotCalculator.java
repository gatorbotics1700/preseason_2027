package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
      Pose2d drivetrainPose,
      ChassisSpeeds drivetrainVelocity,
      Translation3d target,
      double shotSpeed) {
    // calculate field relative shooter pose
    Translation2d botToShooter =
        new Translation2d(Constants.BOT_TO_SHOOTER.getX(), Constants.BOT_TO_SHOOTER.getY());
    Transform2d botToShooterTransform = new Transform2d(botToShooter, new Rotation2d(0));
    Pose2d fieldToShooter2d = drivetrainPose.transformBy(botToShooterTransform);
    Translation3d fieldToShooter =
        new Translation3d(
            fieldToShooter2d.getX(),
            fieldToShooter2d.getY(),
            Constants.BOT_TO_SHOOTER
                .getZ()); // this assumes the bot pose is always 2d and therefore on the floor
    Rotation2d uncompYaw =
        new Rotation2d(
            Math.atan2(
                target.getY() - fieldToShooter.getY(), target.getX() - fieldToShooter.getX()));

    Translation2d botVelo =
        new Translation2d(
            drivetrainVelocity.vxMetersPerSecond, drivetrainVelocity.vxMetersPerSecond);

    Translation2d shooterVelo =
        botVelo; // TODO add math later because shooter velo isn't the same as robot velo
    // if the robot is turning because the turret isn't the center of the
    // rotation
    Translation2d trajectoryRelativeShooterVelo =
        shooterVelo.rotateBy(
            uncompYaw); // uncomp yaw is the angle from the robot to the hub so long as turret
    // zero is robot zero
    double tangentialVelo = trajectoryRelativeShooterVelo.getY();
    double radialVelo = trajectoryRelativeShooterVelo.getX();
    Rotation2d turretAdjust = new Rotation2d(Math.atan2(-tangentialVelo, shotSpeed));
    Rotation2d compYaw = uncompYaw.plus(turretAdjust);
    Rotation2d turretAngle =
        compYaw; // if we don't want shoot on the move then just switch this to uncomp yaw!

    Translation2d shooterToTargetFieldRelative =
        new Translation2d(
            target.getX() - fieldToShooter.getX(), target.getY() - fieldToShooter.getY());
    double uncompRange = shooterToTargetFieldRelative.getNorm();
    double shooterToHubHeight = target.getZ() - fieldToShooter.getZ();
    double shotTime =
        uncompRange
            / (shotSpeed + radialVelo); // technically for shot time to be accurate we only want the
    // horizontal radial velocity (because otherwise the shot angle
    // matters) but it might be fine to approximate like this
    double compRange =
        shotTime * Math.sqrt(tangentialVelo * tangentialVelo + shotSpeed * shotSpeed);
    // Amelia adds ballistics equation here to calculate the angle using compRange and
    // shooterToHubHeight!

    Rotation2d hoodAngle =
        new Rotation2d(
            Math.atan(
                (-2 * Math.pow(shotSpeed, 2) * compRange
                        - Math.sqrt(
                            4 * Math.pow(shotSpeed, 4) * Math.pow(compRange, 2)
                                - 4 * Math.pow(9.8, 2) * Math.pow(compRange, 4)
                                - 8
                                    * 9.8
                                    * Math.pow(compRange, 2)
                                    * Math.pow(shotSpeed, 2)
                                    * shooterToHubHeight))
                    / (-2 * 9.8 * Math.pow(compRange, 2))));
    return new ShotParameters(turretAngle, hoodAngle);
  }
}
