package frc.robot.commands.mech;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.ClimberSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class ClimbCommands {

  private ClimbCommands() {}

  public static Command ExtendClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, ClimberConstants.L1_EXTENSION_INCHES);
  }

  public static Command RetractClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, 0.0);
  }

  private static class HomeClimber extends Command {
    private final ClimberSubsystem climberSubsystem;

    HomeClimber(ClimberSubsystem climberSubsystem) {
      this.climberSubsystem = climberSubsystem;
      addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
      climberSubsystem.setClimberVoltage(ClimberConstants.HOMING_VOLTAGE);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
      return climberSubsystem.limitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
      climberSubsystem.zeroClimber();
      climberSubsystem.setDesiredPositionInches(ClimberConstants.RETRACTED_HEIGHT_INCHES + 0.25);
    }
  }

  // teleop version - uses pose and logic
  public static Command DriveToTower(Drive drive) {
    return new DriveToTowerCommand(drive, null);
  }

  // auto version - uses the alliance from the chooser
  public static Command DriveToTower(Drive drive, String alliance) {
    return new DriveToTowerCommand(drive, alliance);
  }

  /** Teleop version - determines alliance from current pose */
  public static Command Climb(Drive drive, ClimberSubsystem climberSubsystem)
      throws IOException, ParseException {
    return ExtendClimber(climberSubsystem)
        .alongWith(DriveToTower(drive))
        .andThen(RetractClimber(climberSubsystem));
  }

  // auto version - uses the alliance from the chooser
  public static Command Climb(Drive drive, ClimberSubsystem climberSubsystem, String alliance)
      throws IOException, ParseException {
    return ExtendClimber(climberSubsystem)
        .alongWith(DriveToTower(drive, alliance))
        .andThen(RetractClimber(climberSubsystem));
  }

  // auto version that skips DriveToTower - use when already at tower (e.g. after depot-to-tower
  // path)
  public static Command ClimbWithoutDrive(ClimberSubsystem climberSubsystem) {
    return ExtendClimber(climberSubsystem).andThen(RetractClimber(climberSubsystem));
  }

  private static class DriveToTowerCommand extends Command {
    private final Drive drive;
    private final String
        allianceFromChooser; // null means that we will determine from pose (teleop)
    private Command pathCommand;

    DriveToTowerCommand(Drive drive, String allianceFromChooser) {
      this.drive = drive;
      this.allianceFromChooser = allianceFromChooser;
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      PathConstraints constraints =
          new PathConstraints(4, 4, Units.degreesToRadians(500), Units.degreesToRadians(800));

      Pose2d pose = drive.getPose();

      // Determine alliance - use chooser value for auto or determine from pose for teleop
      String allianceToUse = allianceFromChooser;
      if (allianceToUse == null) {
        boolean isBlue = pose.getX() <= FieldCoordinates.FIELD_CENTER.getX();
        allianceToUse = isBlue ? "B" : "R";
      }

      try {
        if (allianceToUse.equals("B")) { // blue
          if (pose.getX() <= FieldCoordinates.BLUE_BUMP_AND_TRENCH_X
              && pose.getY() <= FieldCoordinates.FIELD_CENTER.getY()) {
            pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("B AR to Tower Right"),
                    constraints); // alliance zone bottom
          } else if (pose.getX() <= FieldCoordinates.BLUE_BUMP_AND_TRENCH_X
              && pose.getY() > FieldCoordinates.FIELD_CENTER.getY()) {
            pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("B AL to Tower Left"),
                    constraints); // alliance zone top
          } else if (pose.getX() >= FieldCoordinates.BLUE_BUMP_AND_TRENCH_X
              && pose.getY() <= FieldCoordinates.FIELD_CENTER.getY()) {
            pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("B NR to Tower Right"),
                    constraints); // neutral zone bottom
          } else {
            pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("B NL to Tower Left"),
                    constraints); // neutral zone top
          }
        } else { // red
          if (pose.getX() <= FieldCoordinates.RED_BUMP_AND_TRENCH_X
              && pose.getY() <= FieldCoordinates.FIELD_CENTER.getY()) {
            pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("R NL to Tower Left"),
                    constraints); // neutral zone bottom
          } else if (pose.getX() <= FieldCoordinates.RED_BUMP_AND_TRENCH_X
              && pose.getY() > FieldCoordinates.FIELD_CENTER.getY()) {
            pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("R NR to Tower Right"),
                    constraints); // neutral zone top
          } else if (pose.getX() >= FieldCoordinates.RED_BUMP_AND_TRENCH_X
              && pose.getY() <= FieldCoordinates.FIELD_CENTER.getY()) {
            pathCommand =
                AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("R AL to Tower Left"),
                    constraints); // alliance zone bottom
          } else {
            pathCommand = Commands.none();
          }
        }
      } catch (Exception e) {
        pathCommand = Commands.none();
      }

      if (pathCommand != null) {
        pathCommand.initialize();
      }
    }

    @Override
    public void execute() {
      if (pathCommand != null) {
        pathCommand.execute();
      }
    }

    @Override
    public boolean isFinished() {
      return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
      if (pathCommand != null) {
        pathCommand.end(interrupted);
      }
    }
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
