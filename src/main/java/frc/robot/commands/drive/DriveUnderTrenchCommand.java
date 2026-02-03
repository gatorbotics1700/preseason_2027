package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.HoodRetractCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.HoodSubsystem;

import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveUnderTrenchCommand {

  public static Command driveUnderTrench(Drive drive, HoodSubsystem hoodSubsystem) throws IOException, ParseException {
    PathConstraints constraints =
        new PathConstraints(8, 8, Units.degreesToRadians(700), Units.degreesToRadians(1000));

    Pose2d pose = drive.getPose();
    System.out.println("*********************DRIVE UNDER TRENCH COMMAND*******************");
    
    Command pathToFollow;

    if (pose.getX() <= Constants.FIELD_CENTER.getX()) { // BLUE
      if (pose.getY() < Constants.FIELD_CENTER.getY()
          && pose.getX() < Constants.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B TR N to A"), constraints);
      } else if (pose.getY() > Constants.FIELD_CENTER.getY()
          && pose.getX() < Constants.BLUE_BUMP_AND_TRENCH_X) {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B TL N to A"), constraints);
      } else if ((pose.getY() > Constants.FIELD_CENTER.getY())
          && (pose.getX() > Constants.BLUE_BUMP_AND_TRENCH_X)) {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B TL A to N"), constraints);
      } else {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B TR A to N"), constraints);
      }
    } else { // RED
      if (pose.getY() < Constants.FIELD_CENTER.getY()
          && pose.getX() < Constants.RED_BUMP_AND_TRENCH_X) {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R TL A to N"), constraints);
      } else if (pose.getY() > Constants.FIELD_CENTER.getY()
          && pose.getX() < Constants.RED_BUMP_AND_TRENCH_X) {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R TR A to N"), constraints);
      } else if ((pose.getY() > Constants.FIELD_CENTER.getY())
          && (pose.getX() > Constants.RED_BUMP_AND_TRENCH_X)) {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R TR N to A"), constraints);
      } else {
        pathToFollow = AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R TL N to A"), constraints);
      }
    }
    return new HoodRetractCommand(hoodSubsystem).andThen(pathToFollow); //TODO: stop shooting, retract hood, and then follow path -- add the stop shooting part
  }
}
