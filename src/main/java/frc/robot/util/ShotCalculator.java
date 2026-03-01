package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotCalculatorConditions;
import org.littletonrobotics.junction.Logger;

// @AutoLog
public class ShotCalculator {
  // for testing only, used for logging where the calculator expects the ball to hit the target's
  // height ("land" on the target)
  public static Translation3d landingCoords = new Translation3d();
  public static ShotParameters[][][] lookupTable;

  public ShotCalculator() {
    double elevation = FieldCoordinates.BLUE_HUB.getZ() - ShooterConstants.BOT_TO_SHOOTER.getZ();
    lookupTable = getShootingLookupTable(elevation);
  }

  // This is the method we use to get shot parameters, which returns a hood angle (from vertical),
  // turret angle (robot relative), and a shotspeed (in mps of the ball, not flywheel rpm)
  // It iterates through every combination of hood angles and shotspeeds and returns the combination
  // that produces the highest arc within the height constraints that is within the error deadband
  // If it finds no valid shot (ie. every hood and speed combination has too great an error and/or
  // the arc height does not meet the constraints) it will return the retracted hood position,
  // turret angle zero, and shotspeed zero

  // if it's returning a shotspeed of zero try:
  // moving the robot back
  // making sure your position relative to the target is accurate
  // your hood angle and arc constraints are right

  // I am fairly certain the math itself works because it worked in sim, so if you are having issues
  // with the parameters it is returning it is more likely to be an issue with the inputs than the
  // function.

  public static ShotParameters calculateShot(
      Pose2d drivetrainPose, ChassisSpeeds chassisSpeeds, Translation3d target) {
    System.out.println("CALCULATING SHOT");

    // calculate field relative shooter pose
    Translation3d fieldToShooter =
        getFieldToShooter(drivetrainPose, ShooterConstants.BOT_TO_SHOOTER);

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
        rotateFrameOfReference(fieldRelativeShooterVelo, uncompTurretToTargetAngle.unaryMinus());

    // uncomp yaw is the angle from the robot to the hub so
    // long as
    // turret zero is robot zero and uncomp yaw is field relative

    double tangentialVelo = trajectoryRelativeShooterVelo.getY();
    double radialVelo = trajectoryRelativeShooterVelo.getX();

    double uncompRange = get2dDistance(fieldToShooter, target);

    // double shooterToHubHeight = target.getZ() - fieldToShooter.getZ();
    Translation3d fieldRelativeShooterToTarget = target.minus(fieldToShooter);

    ShotParameters trajectoryRelativeParams =
        lookupShot(lookupTable, tangentialVelo, radialVelo, uncompRange);
    // sweepTrajectories( tangentialVelo, radialVelo, uncompRange, target.getZ() -
    // fieldToShooter.getZ());

    ShotParameters botRelativeParams =
        new ShotParameters(
            trajectoryRelativeParams
                .turretAngle
                .plus(uncompTurretToTargetAngle)
                .minus(drivetrainPose.getRotation()),
            trajectoryRelativeParams.hoodAngle,
            trajectoryRelativeParams.shotSpeed);
    Logger.recordOutput("shotCalculator/turretAngle", botRelativeParams.turretAngle);
    Logger.recordOutput("shotCalculator/landingCoords", landingCoords);
    Logger.recordOutput(
        "shotCalculator/shooterToTarget + shooter",
        fieldRelativeShooterToTarget.plus(fieldToShooter));
    System.out.println("SHOT CALCULATOR DONE");

    return botRelativeParams;
  }

  public static ShotParameters sweepTrajectories(
      double tangentialVelo, double radialVelo, double uncompRange, double elevation) {

    double speedRange = ShotCalculatorConditions.MAX_SHOT_SPEED - 0;
    int speedIterations = (int) (speedRange / 0.5);
    double speedIncrement = speedRange / (double) speedIterations;
    double hoodAngleRange =
        HoodConstants.RETRACTED_POSITION.getDegrees() - HoodConstants.MIN_ANGLE.getDegrees();
    int angleIterations = (int) (hoodAngleRange / 1);
    Rotation2d angleIncrement =
        new Rotation2d(Math.toRadians(hoodAngleRange / (double) angleIterations));

    double highestArc = 0;

    Rotation2d testHoodAngle = HoodConstants.MIN_ANGLE;
    double testShotSpeed = 0;
    Rotation2d testTurretAdjust = new Rotation2d();

    Rotation2d bestTurretAdjust = new Rotation2d();
    Rotation2d bestHoodAngle = new Rotation2d();
    double bestShotSpeed = 0;

    for (int i = 0; i < speedIterations; i++) {
      testHoodAngle = HoodConstants.MIN_ANGLE;

      for (int j = 0; j < angleIterations; j++) {

        double effectiveRadialVelo = testShotSpeed * testHoodAngle.getCos() + radialVelo;
        double shotTime = uncompRange / (effectiveRadialVelo);

        double compRange =
            shotTime
                * Math.sqrt(
                    tangentialVelo * tangentialVelo + effectiveRadialVelo * effectiveRadialVelo);

        testTurretAdjust = new Rotation2d(Math.atan2(-tangentialVelo, effectiveRadialVelo));

        double error =
            getTrajectoryError(
                testHoodAngle,
                testTurretAdjust,
                tangentialVelo,
                radialVelo,
                uncompRange,
                elevation,
                testShotSpeed);

        double apexTime = apexTime(testShotSpeed * Math.sin(testHoodAngle.getRadians()));
        double vertexHeight =
            vertexHeight(
                ShooterConstants.BOT_TO_SHOOTER.getZ(),
                testShotSpeed * Math.sin(testHoodAngle.getRadians()),
                apexTime);

        double vertexRange = vertexRange(effectiveRadialVelo, tangentialVelo, apexTime);

        if (vertexHeight >= ShotCalculatorConditions.MIN_SHOT_HEIGHT
            && vertexHeight <= ShotCalculatorConditions.MAX_SHOT_HEIGHT
            && vertexRange < compRange
            && testShotSpeed <= ShotCalculatorConditions.MAX_SHOT_SPEED
            && Math.abs(error) <= ShotCalculatorConditions.SHOT_DEADBAND
            && vertexHeight > highestArc) {
          bestTurretAdjust = testTurretAdjust;
          bestHoodAngle = testHoodAngle;
          bestShotSpeed = testShotSpeed;
          highestArc = vertexHeight;
          // System.out.println(bestHoodAngle.getDegrees() + ", " + highestArc);
        }

        testHoodAngle = testHoodAngle.plus(angleIncrement);
      }
      testShotSpeed += speedIncrement;
    }

    if (bestShotSpeed == 0) {
      // System.out.println("NO VALID SHOT");
    } else {
      // System.out.println("VALID SHOT CALCULATED");
    }

    return new ShotParameters(bestTurretAdjust, bestHoodAngle, bestShotSpeed);
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
      Rotation2d hoodAngle,
      Rotation2d turretAdjust,
      double tangentialVelo,
      double radialVelo,
      double uncompRange,
      double elevation,
      double shotSpeed) {

    // all of this is in the coordinate space with x pointing from shooter to target (trajectory
    // relative)
    double vx = shotSpeed * Math.cos(hoodAngle.getRadians()) * Math.cos(turretAdjust.getRadians());
    double vy = shotSpeed * Math.cos(hoodAngle.getRadians()) * Math.sin(turretAdjust.getRadians());
    double vz = shotSpeed * Math.sin(hoodAngle.getRadians());

    vx += radialVelo;
    vy += tangentialVelo;

    Translation3d initialVelocity = new Translation3d(vx, vy, vz);

    double t =
        timeToTargetHeight(
            -0.5 * 9.8,
            initialVelocity.getZ(),
            -elevation); // TODO figure out why this is negative? it works but I'm skeptical

    Translation3d trajectoryRelativeShooterToBallLanding =
        new Translation3d(
            initialVelocity.getX() * t,
            initialVelocity.getY() * t,
            initialVelocity.getZ() * t - 0.5 * 9.8 * t * t);

    Translation3d trajectoryRelativeShooterToTarget = new Translation3d(uncompRange, 0, elevation);

    double error =
        trajectoryRelativeShooterToBallLanding.minus(trajectoryRelativeShooterToTarget).getNorm();
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
  }

  public static Translation2d calculateShooterVelo(
      ChassisSpeeds chassisSpeeds, Rotation2d drivetrainHeading) {
    double botToShooterRadius =
        ShooterConstants.BOT_TO_SHOOTER
            .getX(); // NOTE: this only works because the shooter is centered in the y, otherwise we
    // would need to use pythag
    double omegaAdjust = chassisSpeeds.omegaRadiansPerSecond * botToShooterRadius;
    double vxAdjust = omegaAdjust * Math.cos(drivetrainHeading.getRadians());
    double vyAdjust = omegaAdjust * Math.sin(drivetrainHeading.getRadians());
    double vx = chassisSpeeds.vxMetersPerSecond + vxAdjust;
    double vy = chassisSpeeds.vyMetersPerSecond + vyAdjust;
    return new Translation2d(vx, vy);
  }

  public static ShotParameters[][][] getShootingLookupTable(double elevation) {
    System.out.println("GENERATING TABLE");
    double startTime = System.currentTimeMillis();
    int veloIncrements =
        (int)
            (ShotCalculatorConditions.MAX_COMPONENT_VELO
                * 2
                / ShotCalculatorConditions.VELO_INCREMENT); // times 2 to account for negative velo
    int rangeIncrements =
        (int) (ShotCalculatorConditions.MAX_RANGE / ShotCalculatorConditions.RANGE_INCREMENT);
    ShotParameters[][][] lookupTable =
        new ShotParameters[veloIncrements][veloIncrements][rangeIncrements];
    System.out.println("INCREMENTS: " + veloIncrements + ", " + rangeIncrements);
    System.out.println("ITERATIONS: " + veloIncrements * veloIncrements * rangeIncrements);

    double tangentialVelo = 0;
    double radialVelo = 0;
    double range = 0; // TODO add min range perhaps?

    for (int i = 0; i < veloIncrements; i++) {
      for (int j = 0; j < veloIncrements; j++) {
        for (int k = 0; k < rangeIncrements; k++) {
          lookupTable[i][j][k] = sweepTrajectories(tangentialVelo, radialVelo, range, elevation);
          range += ShotCalculatorConditions.RANGE_INCREMENT;
        }
        radialVelo += ShotCalculatorConditions.VELO_INCREMENT;
      }
      tangentialVelo += ShotCalculatorConditions.VELO_INCREMENT;
    }
    double endTime = System.currentTimeMillis();
    System.out.println("TABLE GENERATED IN " + (endTime - startTime) + " MILIS");
    return lookupTable;
  }

  public static ShotParameters lookupShot(
      ShotParameters[][][] lookupTable, double tangentialVelo, double radialVelo, double range) {
    if (range > ShotCalculatorConditions.MAX_RANGE) {
      System.out.println("OUT OF RANGE OUT OF RANGE");
    }
    if (Math.abs(radialVelo) > ShotCalculatorConditions.MAX_COMPONENT_VELO) {
      System.out.println("TOO FAST R TOO FAST R");
    }
    if (Math.abs(tangentialVelo) > ShotCalculatorConditions.MAX_COMPONENT_VELO) {
      System.out.println("TOO FAST T TOO FAST T");
    }
    int i =
        (int)
            ((tangentialVelo + ShotCalculatorConditions.MAX_COMPONENT_VELO)
                / ShotCalculatorConditions.VELO_INCREMENT);
    int j =
        (int)
            ((radialVelo + ShotCalculatorConditions.MAX_COMPONENT_VELO)
                / ShotCalculatorConditions.VELO_INCREMENT);
    int k = (int) (range / ShotCalculatorConditions.RANGE_INCREMENT);
    ShotParameters params = lookupTable[i][j][k];
    System.out.println(params.shotSpeed + ", " + params.hoodAngle + ", " + params.turretAngle);
    return params;
  }
}
