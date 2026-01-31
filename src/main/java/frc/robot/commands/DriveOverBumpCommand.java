package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveOverBumpCommand {

  private static final Translation2d CENTER = new Translation2d(8.270494, 4.034663);
  private static final double BLUE_BUMP_X = 4.626;
  private static final double RED_BUMP_X = 11.915;

  public static Command driveOverBump(Drive drive) throws IOException, ParseException {
    Pose2d pose = drive.getPose();
    System.out.println("*********************DRIVE OVER BUMP COMMAND*******************");

    if (pose.getX() < CENTER.getX()) {
      if (pose.getX() > BLUE_BUMP_X && pose.getY() < CENTER.getY()) {
        // B BR N to A
        System.out.println("B BR N to A");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("B BR N to A"));
      } else if (pose.getX() > BLUE_BUMP_X && pose.getY() > CENTER.getY()) {
        // B BL N to A
        System.out.println("B BL N to A");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("B BL N to A"));
      } else if (pose.getX() < BLUE_BUMP_X && pose.getY() < CENTER.getY()) {
        // B BR A to N
        System.out.println("B BR A to N");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("B BR A to N"));
      } else if (pose.getX() < BLUE_BUMP_X && pose.getY() > CENTER.getY()) {
        // B BL A to N
        System.out.println("B BL A to N");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("B BL A to N"));
      }

    } else {
      if (pose.getX() < RED_BUMP_X && pose.getY() < CENTER.getY()) {
        // R BL N to A
        System.out.println("R BL N to A");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("R BL N to A"));
      } else if (pose.getX() < RED_BUMP_X && pose.getY() > CENTER.getY()) {
        // R BR N to A
        System.out.println("R BR N to A");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("R BR N to A"));
      } else if (pose.getX() > RED_BUMP_X && pose.getY() < CENTER.getY()) {
        // R BL A to N
        System.out.println("R BL A to N");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("R BL A to N"));
      } else if (pose.getX() > RED_BUMP_X && pose.getY() > CENTER.getY()) {
        // R BR A to N
        System.out.println("R BR A to N");
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("R BR A to N"));
      }
    }
    return null;
  }
}
