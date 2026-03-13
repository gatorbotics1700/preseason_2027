package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.ShooterSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveUnderTrenchCommand {

  public static Command driveUnderTrench(Drive drive, ShooterSubsystem shooterSubsystem)
      throws IOException, ParseException {
    PathConstraints constraints =
        new PathConstraints(8, 8, Units.degreesToRadians(700), Units.degreesToRadians(1000));

    Pose2d pose = drive.getPose();
    System.out.println("*********************DRIVE UNDER TRENCH COMMAND*******************");

    Command pathToFollow;

    if (pose.getX() <= FieldCoordinates.FIELD_CENTER.getX()) { // BLUE
      if (pose.getY() < FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TR A to N"), constraints);
        shooterSubsystem.setShouldShoot(false);
      } else if (pose.getY() > FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TL A to N"), constraints);
        shooterSubsystem.setShouldShoot(false);
      } else if ((pose.getY() > FieldCoordinates.FIELD_CENTER.getY())
          && (pose.getX() > FieldCoordinates.BLUE_BUMP_AND_TRENCH_X)) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TL N to A"), constraints);
        shooterSubsystem.setShouldShoot(false);
      } else {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("B TR N to A"), constraints);
        shooterSubsystem.setShouldShoot(false);
      }
    } else { // RED
      if (pose.getY() < FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TL N to A"), constraints);
        shooterSubsystem.setShouldShoot(false);
      } else if (pose.getY() > FieldCoordinates.FIELD_CENTER.getY()
          && pose.getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TR N to A"), constraints);
        shooterSubsystem.setShouldShoot(false);
      } else if ((pose.getY() > FieldCoordinates.FIELD_CENTER.getY())
          && (pose.getX() > FieldCoordinates.RED_BUMP_AND_TRENCH_X)) {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TR A to N"), constraints);
        shooterSubsystem.setShouldShoot(false);
      } else {
        pathToFollow =
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("R TL A to N"), constraints);
        shooterSubsystem.setShouldShoot(false);
      }
    }
    return pathToFollow.withName("DriveUnderTrench");
  }
}
