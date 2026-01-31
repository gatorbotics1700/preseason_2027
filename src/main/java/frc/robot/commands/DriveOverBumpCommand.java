package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveOverBumpCommand {

  private static final Translation2d CENTER = new Translation2d(8.270494, 4.034663);
  private static final double BLUE_BUMP_X = 4.626;
  private static final double RED_BUMP_X = 11.915;

  public static Command driveOverBump(Drive drive) throws IOException, ParseException {
    PathConstraints constraints =
        new PathConstraints(8, 8, Units.degreesToRadians(700), Units.degreesToRadians(1000));

    Pose2d pose = drive.getPose();
    System.out.println("*********************DRIVE OVER BUMP COMMAND*******************");

    if (pose.getX() <= CENTER.getX()) { // BLUE
      if (pose.getY() < CENTER.getY() && pose.getX() < BLUE_BUMP_X) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B BR N to A"), constraints);
      } else if (pose.getY() > CENTER.getY() && pose.getX() < BLUE_BUMP_X) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B BL N to A"), constraints);
      } else if ((pose.getY() > CENTER.getY()) && (pose.getX() > BLUE_BUMP_X)) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B BL A to N"), constraints);
      } else {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B BR A to N"), constraints);
      }
    } else { // RED
      if (pose.getY() < CENTER.getY() && pose.getX() < RED_BUMP_X) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R BL A to N"), constraints);
      } else if (pose.getY() > CENTER.getY() && pose.getX() < RED_BUMP_X) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R BR A to N"), constraints);
      } else if ((pose.getY() > CENTER.getY()) && (pose.getX() > RED_BUMP_X)) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R BR N to A"), constraints);
      } else {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R BL N to A"), constraints);
      }
    }
  }
}
