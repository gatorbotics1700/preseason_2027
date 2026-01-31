package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.drive.Drive;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class DriveOverBumpCommand {

  // y value to split top vs bottom half of field
  private static final double BUMP_Y = 4.034663; // METERSSSSSS
  // x value to split left vs right half of field
  private static final double BUMP_X = 8.270494; // meters

  public static Command driveOverBump(Drive drive) throws IOException, ParseException {
    return new ConditionalCommand(
        // y < BUMP_Y (bottom half) — go to top me thinks
        new ConditionalCommand(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("B BR A to N")),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("B BL A to N")),
            () -> drive.getPose().getX() < BUMP_X),
        // Y >= BUMP_Y (top half) — go to bottom me thinks
        new ConditionalCommand(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("R BL A to N")),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("R BL to A N")),
            () -> drive.getPose().getX() < BUMP_X),
        () -> drive.getPose().getY() < BUMP_Y);
  }
}
