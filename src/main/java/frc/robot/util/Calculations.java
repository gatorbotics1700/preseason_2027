package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Calculations {

  public static Rotation2d angleToPoint(double deltaX, double deltaY) {
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }
}
