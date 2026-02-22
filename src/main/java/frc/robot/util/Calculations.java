package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Calculations {

  public static Rotation2d angleToPoint(double deltaX, double deltaY) {
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }

  public static double distanceToPoseInMeters(Pose2d firstPose, Pose2d secondPose) {
    return Math.sqrt(
        Math.pow(firstPose.getX() - secondPose.getX(), 2)
            + Math.pow(firstPose.getY() - secondPose.getY(), 2));
  }
}
