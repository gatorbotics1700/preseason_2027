package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

// this is what we will use in constant tracking command!

@AutoLog
public class ShotCalculator {
  public static double lastError = 20;
  public static int loopCount = 0;

  public static Rotation2d hoodAngleGuess = new Rotation2d(Math.toRadians(45));

  public ShotCalculator() {}

  public static ShotParameters calculateShot(
      Pose2d drivetrainPose,
      ChassisSpeeds drivetrainVelocity,
      Translation3d target,
      double shotSpeed) {
    // calculate field relative shooter pose
    Translation3d fieldToShooter = getFieldToShooter(drivetrainPose, Constants.BOT_TO_SHOOTER);

    Rotation2d uncompTurretToTargetAngle = getFieldRelativeYaw(fieldToShooter, target);

    Translation2d drivetrainVelo = // robot relative
        new Translation2d(
            drivetrainVelocity.vxMetersPerSecond, drivetrainVelocity.vyMetersPerSecond);

    Translation2d shooterVelo =
        drivetrainVelo; // TODO add math later because shooter velo isn't the same as robot velo

    Translation2d fieldRelativeShooterVelo =
        rotateFrameOfReference(shooterVelo, drivetrainPose.getRotation().unaryMinus());
    // if the robot is turning because the turret isn't the center of the
    // rotation
    Translation2d trajectoryRelativeShooterVelo =
        rotateFrameOfReference(
            fieldRelativeShooterVelo,
            uncompTurretToTargetAngle.unaryMinus()); // might not need to be inverted, double check
    // uncomp yaw is the angle from the robot to the hub so
    // long as
    // turret zero is robot zero and uncomp yaw is field relative

    double tangentialVelo = trajectoryRelativeShooterVelo.getY();
    double radialVelo = trajectoryRelativeShooterVelo.getX();

    double uncompRange = get2dDistance(fieldToShooter, target);

    double shooterToHubHeight = target.getZ() - fieldToShooter.getZ();

    // Turret lead: offset needed = tangentialVelo * shotTime, so tan(lead) = tangentialVelo /
    // (shotSpeed+radialVelo)
    double effectiveRadialVelo = shotSpeed * hoodAngleGuess.getCos() + radialVelo;

    Rotation2d turretAdjust = new Rotation2d(Math.atan2(-tangentialVelo, effectiveRadialVelo));
    Rotation2d compTurretToTargetAngle =
        uncompTurretToTargetAngle.plus(turretAdjust); // field relative
    Rotation2d turretAngle =
        compTurretToTargetAngle.minus(drivetrainPose.getRotation()); // robot relative

    double shotTime =
        uncompRange
            / (radialVelo
                + shotSpeed); // this probably needs a lookup table because shotSpeed isn't the same
    // direction as radial velo

    // Compensated range (for aim point): shotTime * sqrt(tangential^2 + shotSpeed^2)
    double compRange =
        shotTime * Math.sqrt(tangentialVelo * tangentialVelo + shotSpeed * shotSpeed);

    Rotation2d hoodAngle = solveBallistics(compRange, shooterToHubHeight, shotSpeed);
    double error =
        getTrajectoryError(
            compTurretToTargetAngle,
            hoodAngle,
            fieldRelativeShooterVelo,
            fieldToShooter,
            shotSpeed,
            target);

    Translation2d compBotToTarget = new Translation2d(compRange, turretAngle);
    Pose2d compFieldToTarget =
        drivetrainPose.transformBy(new Transform2d(compBotToTarget, new Rotation2d()));

    Logger.recordOutput(
        "shotCalculator/trajectoryRelativeShooterVelo", trajectoryRelativeShooterVelo);
    Logger.recordOutput("shotCalculator/hoodAngle", hoodAngle.getDegrees());
    Logger.recordOutput("shotCalculator/turretAngle", turretAngle);
    Logger.recordOutput("shotCalculator/turretAdjust", turretAdjust.getDegrees());
    Logger.recordOutput("shotCalculator/compRangeAdjust", compRange - uncompRange);
    Logger.recordOutput("shotCalculator/compRange", compRange);
    Logger.recordOutput("shotCalculator/shotTime", shotTime);
    Logger.recordOutput("shotCalculator/compFieldToTarget", compFieldToTarget);

    if (loopCount > 9) {
      System.out.println("SHOT NOT POSSIBLE");
      lastError = 20;
      loopCount = 0;
      return new ShotParameters(turretAngle, hoodAngle);
    }
    if (Math.abs(error) > 0.02) {
      if (Math.abs(lastError) < Math.abs(error)) {
        System.out.println("ERROR GETTING BIGGER ERROR GETTING BIGGER");
        // lastError = 20;
        // loopCount = 0;
        // return new ShotParameters(turretAngle, hoodAngle);
      }
      lastError = error;
      loopCount++;
      System.out.println("SHOT CALCULATOR COMPLETED LOOP " + loopCount);
      Rotation2d hoodAdjust = hoodAngle.minus(hoodAngleGuess);
      if (error < 0) {
        hoodAngleGuess = hoodAngleGuess.minus(hoodAdjust);
      } else {
        hoodAngleGuess = hoodAngleGuess.plus(hoodAdjust);
      }
      return calculateShot(drivetrainPose, drivetrainVelocity, target, shotSpeed);
    } else {
      lastError = 20;
      loopCount = 0;
      return new ShotParameters(turretAngle, hoodAngle);
    }
  }

  /** Field-frame 3D position of the shooter exit given the robot pose. */
  public static Translation3d getFieldToShooter(
      Pose2d fieldToDrivetrain, Translation3d drivetrainToShooter) {
    Pose3d fieldToDrivetrain3d =
        new Pose3d(
            fieldToDrivetrain.getX(),
            fieldToDrivetrain.getY(),
            0,
            new Rotation3d(fieldToDrivetrain.getRotation()));
    Pose3d fieldToShooter =
        fieldToDrivetrain3d.transformBy(new Transform3d(drivetrainToShooter, new Rotation3d()));
    return fieldToShooter.getTranslation();
  }

  public static Rotation2d getFieldRelativeYaw(Translation3d from, Translation3d to) {
    return new Rotation2d(Math.atan2(to.getY() - from.getY(), to.getX() - from.getX()));
  }

  public static Translation2d rotateFrameOfReference(Translation2d vector, Rotation2d angle) {
    return vector.rotateBy(angle);
  }

  public static double get2dDistance(Translation3d from, Translation3d to) {
    Translation2d vector = new Translation2d(to.getX() - from.getX(), to.getY() - from.getY());
    return vector.getNorm();
  }

  public static Rotation2d solveBallistics(
      double rangeForHood, double shooterToHubHeight, double shotSpeed) {
    return new Rotation2d(
        Math.atan(
            (-2 * Math.pow(shotSpeed, 2) * rangeForHood
                    - Math.sqrt(
                        4 * Math.pow(shotSpeed, 4) * Math.pow(rangeForHood, 2)
                            - 4 * Math.pow(9.8, 2) * Math.pow(rangeForHood, 4)
                            - 8
                                * 9.8
                                * Math.pow(rangeForHood, 2)
                                * Math.pow(shotSpeed, 2)
                                * shooterToHubHeight))
                / (-2 * 9.8 * Math.pow(rangeForHood, 2))));
  }

  public static double getTrajectoryError(
      Rotation2d compTurretToTargetAngle,
      Rotation2d hoodAngle,
      Translation2d fieldRelativeShooterVelo,
      Translation3d fieldToShooter,
      double shotSpeed,
      Translation3d target) {

    double vx = shotSpeed * Math.cos(hoodAngle.getRadians()) * compTurretToTargetAngle.getCos();
    double vy = shotSpeed * Math.cos(hoodAngle.getRadians()) * compTurretToTargetAngle.getSin();
    double vz = shotSpeed * Math.sin(hoodAngle.getRadians());

    vx += fieldRelativeShooterVelo.getX();
    vy += fieldRelativeShooterVelo.getY();

    Translation3d initialVelocity = new Translation3d(vx, vy, vz);
    // Ballistic solution is high-angle (hit on the way down). Use larger root so plotted impact
    // matches.
    double t =
        timeToTargetHeight(
            -0.5 * 9.8, initialVelocity.getZ(), fieldToShooter.getZ() - target.getZ(), true);

    Translation3d ballCoords =
        new Translation3d(
            fieldToShooter.getX() + initialVelocity.getX() * t,
            fieldToShooter.getY() + initialVelocity.getY() * t,
            fieldToShooter.getZ() + initialVelocity.getZ() * t - 0.5 * 9.8 * t * t);

    double error =
        ballCoords.minus(fieldToShooter).getNorm() - target.minus(fieldToShooter).getNorm();
    // Translation2d error =
    //     new Translation2d(ballCoords.getX() - target.getX(), ballCoords.getY() - target.getY());
    return error;
  }

  /**
   * Solves a*t^2 + b*t + c = 0 and returns one positive root.
   *
   * @param useLargerRoot true = return larger positive root (ball at target height on the way down,
   *     matches high-angle ballistic solution); false = return smaller positive root (first
   *     crossing on the way up).
   */
  public static double timeToTargetHeight(double a, double b, double c, boolean useLargerRoot) {
    double sqrt = Math.sqrt(b * b - 4 * a * c);
    double t1 = (-b + sqrt) / (2 * a);
    double t2 = (-b - sqrt) / (2 * a);
    if (t1 > 0 && t2 > 0) {
      return useLargerRoot ? Math.max(t1, t2) : Math.min(t1, t2);
    }
    return t1 > 0 ? t1 : t2;
  }

  /** Returns the first (smaller) positive time when the ball reaches the target height. */
  public static double quadraticEquation(double a, double b, double c) {
    return timeToTargetHeight(a, b, c, false);
  }
}
