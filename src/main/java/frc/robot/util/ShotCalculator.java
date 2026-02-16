package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
  public static double SHOT_DEADBAND = 0.1;
  public static double MIN_SHOT_HEIGHT = 2;
  public static double MAX_SHOT_HEIGHT = 10;
  public static double MIN_SHOT_SPEED = 0;
  public static double MAX_SHOT_SPEED = 25;
  public static Rotation2d MIN_HOOD_ANGLE = new Rotation2d(Math.toRadians(45));
  public static Rotation2d MAX_HOOD_ANGLE = new Rotation2d(Math.toRadians(90));
  public static double lastError = 20;
  public static int loopCount = 0;

  public static double speedRange = MAX_SHOT_SPEED - MIN_SHOT_SPEED;
  public static int speedIterations = (int) (speedRange / 0.5);
  public static double speedIncrement = speedRange / (double) speedIterations;
  public static double hoodAngleRange = MAX_HOOD_ANGLE.getDegrees() - MIN_HOOD_ANGLE.getDegrees();
  public static int angleIterations = (int) (hoodAngleRange / 1);
  public static Rotation2d angleIncrement =
      new Rotation2d(Math.toRadians(hoodAngleRange / (double) angleIterations));

  public static double shotSpeed = MIN_SHOT_SPEED;
  public static Rotation2d turretAngle;
  public static Rotation2d hoodAngle = MIN_HOOD_ANGLE;

  // for testing only
  public static Translation3d landingCoords = new Translation3d();

  public ShotCalculator() {}

  public static ShotParameters calculateShot(
      Pose2d drivetrainPose, ChassisSpeeds chassisSpeeds, Translation3d target) {
    // calculate field relative shooter pose
    Translation3d fieldToShooter = getFieldToShooter(drivetrainPose, Constants.BOT_TO_SHOOTER);

    Rotation2d uncompTurretToTargetAngle = getFieldRelativeYaw(fieldToShooter, target);

    Translation2d shooterVelo =
        calculateShooterVelo(
            chassisSpeeds,
            drivetrainPose
                .getRotation()); // TODO check math! kind of hard to check in sim because to
    // simulate this I do the exact same math on the simulator so it's
    // self affirming :/

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

    ShotParameters params =
        sweepTrajectories(
            tangentialVelo,
            radialVelo,
            uncompRange,
            drivetrainPose,
            uncompTurretToTargetAngle,
            shooterToHubHeight,
            fieldRelativeShooterVelo,
            fieldToShooter,
            target);

    Logger.recordOutput("shotCalculator/turretAngle", params.turretAngle);
    Logger.recordOutput("shotCalculator/landingCoords", landingCoords);

    return params;
  }

  public static ShotParameters sweepTrajectories(
      double tangentialVelo,
      double radialVelo,
      double uncompRange,
      Pose2d drivetrainPose,
      Rotation2d uncompTurretToTargetAngle,
      double shooterToHubHeight,
      Translation2d fieldRelativeShooterVelo,
      Translation3d fieldToShooter,
      Translation3d target) {

    double highestArc = 0;

    Rotation2d testHoodAngle = MIN_HOOD_ANGLE;
    double testShotSpeed = MIN_SHOT_SPEED;
    Rotation2d testTurretAngle = new Rotation2d();

    Rotation2d bestTurretAngle = new Rotation2d();
    Rotation2d bestHoodAngle = new Rotation2d();
    double bestShotSpeed = 0;

    for (int i = 0; i < speedIterations; i++) {
      testHoodAngle = MIN_HOOD_ANGLE;

      for (int j = 0; j < angleIterations; j++) {

        double effectiveRadialVelo = testShotSpeed * testHoodAngle.getCos() + radialVelo;
        double shotTime = uncompRange / (effectiveRadialVelo);

        double compRange =
            shotTime
                * Math.sqrt(
                    tangentialVelo * tangentialVelo + effectiveRadialVelo * effectiveRadialVelo);

        Rotation2d turretAdjust = new Rotation2d(Math.atan2(-tangentialVelo, effectiveRadialVelo));
        Rotation2d compTurretToTargetAngle =
            uncompTurretToTargetAngle.plus(turretAdjust); // field relative
        testTurretAngle =
            compTurretToTargetAngle.minus(drivetrainPose.getRotation()); // robot relative

        double error =
            getTrajectoryError(
                compTurretToTargetAngle,
                testHoodAngle,
                fieldRelativeShooterVelo,
                fieldToShooter,
                testShotSpeed,
                target);

        double apexTime = apexTime(testShotSpeed * Math.sin(testHoodAngle.getRadians()));
        double vertexHeight =
            vertexHeight(
                fieldToShooter.getZ(),
                testShotSpeed * Math.sin(testHoodAngle.getRadians()),
                apexTime);

        double vertexRange = vertexRange(effectiveRadialVelo, tangentialVelo, apexTime);

        if (vertexHeight >= MIN_SHOT_HEIGHT
            && vertexHeight <= MAX_SHOT_HEIGHT
            && vertexRange < compRange
            && shotSpeed <= MAX_SHOT_SPEED
            && Math.abs(error) <= SHOT_DEADBAND
            && vertexHeight > highestArc) {
          bestTurretAngle = testTurretAngle;
          bestHoodAngle = testHoodAngle;
          bestShotSpeed = testShotSpeed;
          highestArc = vertexHeight;
          System.out.println(bestHoodAngle.getDegrees() + ", " + highestArc);
        }

        testHoodAngle = testHoodAngle.plus(angleIncrement);
      }
      testShotSpeed += speedIncrement;
    }

    return new ShotParameters(bestTurretAngle, bestHoodAngle, bestShotSpeed);
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

    double t =
        timeToTargetHeight(
            -0.5 * 9.8, initialVelocity.getZ(), fieldToShooter.getZ() - target.getZ());

    Translation3d fieldToBall =
        new Translation3d(
            fieldToShooter.getX() + initialVelocity.getX() * t,
            fieldToShooter.getY() + initialVelocity.getY() * t,
            fieldToShooter.getZ() + initialVelocity.getZ() * t - 0.5 * 9.8 * t * t);

    double error = fieldToBall.minus(target).getNorm();
    return error;
  }

  public static double apexTime(double vz) {
    return (-vz) / (2 * (-0.5 * 9.8));
  }

  public static double vertexHeight(double startingHeight, double vz, double vertexT) {
    return startingHeight + vz * vertexT - 0.5 * 9.8 * vertexT * vertexT;
  }

  public static double vertexRange(
      double effectiveRadialVelo, double tangentialVelo, double vertexT) {
    double horizontalSpeed =
        Math.sqrt(effectiveRadialVelo * effectiveRadialVelo + tangentialVelo * tangentialVelo);
    return horizontalSpeed * vertexT;
  }

  public static double timeToTargetHeight(double a, double b, double c) {
    double sqrt = Math.sqrt(b * b - 4 * a * c);
    return (-b - sqrt) / (2 * a);
    // double t1 = (-b + sqrt) / (2 * a);
    // double t2 = (-b - sqrt) / (2 * a);
    // if (t1 > 0 && t2 > 0) {
    //   return Math.max(t1, t2); // returns the larger root, which is the one on the way down
    // }
    // return t1 > 0 ? t1 : t2;
  }

  public static Translation2d calculateShooterVelo(
      ChassisSpeeds chassisSpeeds, Rotation2d drivetrainHeading) {
    double botToShooterRadius =
        Constants.BOT_TO_SHOOTER
            .getX(); // NOTE: this only works because the shooter is centered in the y, otherwise we
    // would need to use pythag
    double omegaAdjust = chassisSpeeds.omegaRadiansPerSecond * botToShooterRadius;
    double vxAdjust = omegaAdjust * Math.cos(drivetrainHeading.getRadians());
    double vyAdjust = omegaAdjust * Math.sin(drivetrainHeading.getRadians());
    double vx = chassisSpeeds.vxMetersPerSecond + vxAdjust;
    double vy = chassisSpeeds.vyMetersPerSecond + vyAdjust;
    return new Translation2d(vx, vy);
  }
}
