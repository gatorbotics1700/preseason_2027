package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.ClimberSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class ClimbCommands {

  private static final double L1_EXTENSION_INCHES = 20; // TODO get a real number

  private ClimbCommands() {}

  public static Command ExtendClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, L1_EXTENSION_INCHES);
  }

  public static Command RetractClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, 0.0);
  }

  public static Command DriveToTower(Drive drive) throws IOException, ParseException {
    PathConstraints constraints =
        new PathConstraints(4, 4, Units.degreesToRadians(500), Units.degreesToRadians(800));

    Pose2d pose = drive.getPose();

    // TODO: can we just pathfindtopose
    // alliance selections
    if (pose.getX() <= Constants.FIELD_CENTER.getX()) { // blue
      if (pose.getX() <= Constants.BLUE_BUMP_AND_TRENCH_X
          && pose.getY() <= Constants.FIELD_CENTER.getY()) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("NAME"), constraints); // TODO: alliance zone bottem
      } else if (pose.getX() <= Constants.BLUE_BUMP_AND_TRENCH_X
          && pose.getY() > Constants.FIELD_CENTER.getY()) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B AL to Tower Left"),
            constraints); // TODO: alliance zone top
      } else if (pose.getX() >= Constants.BLUE_BUMP_AND_TRENCH_X
          && pose.getY() <= Constants.FIELD_CENTER.getY()) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("NAME"), constraints); // TODO: neutral zone bottem

      } else {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("B NL to Tower Left"),
            constraints); // TODO: neutral zone top
      }

    } else { // red

      if (pose.getX() <= Constants.RED_BUMP_AND_TRENCH_X
          && pose.getY() <= Constants.FIELD_CENTER.getY()) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R NL to Tower Left"), constraints); // neutral zone bottem
      } else if (pose.getX() <= Constants.RED_BUMP_AND_TRENCH_X
          && pose.getY() > Constants.FIELD_CENTER.getY()) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("NAME"), constraints); // TODO: neutral zone top
      } else if (pose.getX() >= Constants.RED_BUMP_AND_TRENCH_X
          && pose.getY() <= Constants.FIELD_CENTER.getY()) {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("R AL to Tower Left"),
            constraints); // TODO: alliance zone bottem

      } else {
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("NAME"), constraints); // TODO: alliance zone top
      }
    }
  }

  public static Command Climb(Drive drive, ClimberSubsystem climberSubsystem)
      throws IOException, ParseException {
    return ExtendClimber(climberSubsystem)
        .alongWith(DriveToTower(drive))
        .andThen(RetractClimber(climberSubsystem));
  }

  private static class ClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final double height;

    ClimberCommand(ClimberSubsystem climberSubsystem, double height) {
      this.climberSubsystem = climberSubsystem;
      this.height = height;
      addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
      climberSubsystem.setDesiredPositionInches(height);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
      if (climberSubsystem.currentPositionInches() == 0) { // TODO probably add a deadband here?
        return true;
      }
      return false;
    }
  }
}
