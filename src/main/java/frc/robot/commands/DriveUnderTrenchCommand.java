package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveUnderTrenchCommand {

  // y value to split top vs bottom half of field
  private static final double TRENCH_Y = 4.0;
  // x value to split left vs right half of field
  private static final double TRENCH_X = 4.0;

  public static Command driveUnderTrench(Drive drive) throws IOException, ParseException {
    return new ConditionalCommand(
        // y < BUMP_Y (bottom half) — go to top me thinks
        new ConditionalCommand(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("TODO")),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("TODO")),
            () -> drive.getPose().getX() < TRENCH_X),
        // Y >= BUMP_Y (top half) — go to bottom me thinks
        new ConditionalCommand(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("")),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("")),
            () -> drive.getPose().getX() < TRENCH_X),
        () -> drive.getPose().getY() < TRENCH_Y);
  }
}
