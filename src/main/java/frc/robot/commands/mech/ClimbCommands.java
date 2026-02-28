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
import frc.robot.util.MultiStepAutoChooser;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class ClimbCommands {

  private ClimbCommands() {}

  public static Command ExtendClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, ClimberConstants.L1_EXTENSION_INCHES)
        .withName("ExtendClimber");
  }

  public static Command RetractClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, 0.0).withName("RetractClimber");
  }

  public static class HomeClimber extends Command {
    private final ClimberSubsystem climberSubsystem;

    HomeClimber(ClimberSubsystem climberSubsystem) {
      setName("HomeClimber");
      this.climberSubsystem = climberSubsystem;
      addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
      climberSubsystem.setClimberVoltage(ClimberConstants.HOMING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
      return climberSubsystem.limitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
      // TODO return this to its real state!
      // climberSubsystem.zeroClimber();
      // climberSubsystem.setDesiredPositionInches(ClimberConstants.RETRACTED_HEIGHT_INCHES + 0.25);
      climberSubsystem.setDesiredPositionInches(climberSubsystem.getCurrentPositionInches() + 0.1);
    }
  }

  public static Command Climb(Drive drive, ClimberSubsystem climberSubsystem)
      throws IOException, ParseException {
    return ExtendClimber(climberSubsystem)
        .alongWith(DriveToTower(drive))
        .andThen(RetractClimber(climberSubsystem))
        .withName("Climb");
  }

  public static Command DriveToTower(Drive drive) {
    return new DriveToTowerCommand(drive);
  }

  public static Command ClimbWithoutDrive(ClimberSubsystem climberSubsystem) {
    return ExtendClimber(climberSubsystem)
        .andThen(RetractClimber(climberSubsystem))
        .withName("ClimbWithoutDrive");
  }

  private static class DriveToTowerCommand extends Command {
    private final Drive drive;
    private Command pathCommand;

    DriveToTowerCommand(Drive drive) {
      setName("DriveToTower");
      this.drive = drive;
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      PathConstraints constraints =
          new PathConstraints(4, 4, Units.degreesToRadians(500), Units.degreesToRadians(800));

      Pose2d pose = drive.getPose();

      String allianceToUse = MultiStepAutoChooser.getAlliance();

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
    public boolean isFinished() {
      if (Math.abs(
              climberSubsystem.getCurrentPositionInches()
                  - climberSubsystem.getDesiredPositionInches())
          <= ClimberConstants.POSITION_DEADBAND) {
        return true;
      }
      return false;
    }
  }
}
