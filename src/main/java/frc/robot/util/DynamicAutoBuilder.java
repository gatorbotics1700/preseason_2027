package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.ClimberSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class DynamicAutoBuilder {

  /* TODO: thoughts
   *    deploy and start intake unless first target is tower
   *    do we need to retract intake to climb? -- if so, just add to climb command
   *    how do we test this? -- tried testing in sim and made it so start pose updates properly, paths arent quite running correctly, and idk when mech is trying to run
   */

  private final IntakeSubsystem intakeSubsystem;
  private final Drive drive;
  private final ClimberSubsystem climberSubsystem;

  public DynamicAutoBuilder(
      IntakeSubsystem intakeSubsystem, Drive drive, ClimberSubsystem climberSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.drive = drive;
    this.climberSubsystem = climberSubsystem;
  }

  private String convertFromLocation(String location) {
    // No conversion needed - the chooser already outputs the correct format
    // (e.g., "Fuel Pile L", "DC", "Outpost")
    return location;
  }

  private boolean isInvalid(String value) {
    return value == null || value.equals("None");
  }

  private String buildPathName(String alliance, String from, String to) {
    // Need all three values to build a path name
    if (isInvalid(alliance) || isInvalid(from) || isInvalid(to)) {
      return null;
    }

    String convertedFrom = convertFromLocation(from);
    return alliance + " " + convertedFrom + " to " + to;
  }

  private Command getActionForDestination(String destination) {
    if (destination == null || destination.equals("None")) {
      return Commands.none();
    }

    if (destination.startsWith("D") || destination.startsWith("Fuel Pile")) {
      return IntakeCommands.DeployIntake(intakeSubsystem)
          .andThen(IntakeCommands.RunIntake(intakeSubsystem));
    }

    return Commands.none();
  }

  private Command loadPathCommand(String alliance, String from, String to) {
    String pathName = buildPathName(alliance, from, to);
    if (pathName == null) {
      return null;
    }

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      System.out.println("  Loaded path: " + pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.println("  Path not available: " + pathName + " - skipping");
      return Commands.none();
    }
  }

  private Command loadDepotToTowerPath(String alliance, String depotLocation) {
    // All depots go to Tower Left based on existing path naming
    String pathName = alliance + " " + depotLocation + " to Tower Left";

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      System.out.println("  Loaded depot-to-tower path: " + pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.println("  Depot-to-tower path not available: " + pathName);
      return null;
    }
  }

  /** Checks if the location is a depot (DR, DC, or DL). */
  private boolean isDepot(String location) {
    return location != null
        && (location.equals("DR") || location.equals("DC") || location.equals("DL"));
  }

  public Command buildAuto(
      String alliance, String startPos, String dest1, String dest2, String dest3, boolean climb) {

    System.out.println("============BUILD AUTO============");
    System.out.println("Alliance: " + alliance);
    System.out.println("First Destination: " + dest1);
    System.out.println("Second Destination: " + dest2);
    System.out.println("Third Destination: " + dest3);
    System.out.println("Climb: " + climb);

    if (alliance == null || alliance.equals("None")) {
      System.out.println("DynamicAutoBuilder: Missing alliance");
      return Commands.none();
    }
    if (startPos == null || startPos.equals("None")) {
      System.out.println("DynamicAutoBuilder: Missing start position");
      return Commands.none();
    }

    boolean hasDestination = dest1 != null && !dest1.equals("None");
    if (!hasDestination && !climb) {
      System.out.println("DynamicAutoBuilder: No destinations or climb selected");
      return Commands.none();
    }

    System.out.println(
        "DynamicAutoBuilder: Building auto - "
            + alliance
            + " "
            + startPos
            + " -> "
            + dest1
            + " -> "
            + dest2
            + " -> "
            + dest3
            + (climb ? " -> Tower" : ""));

    List<Command> commandSequence = new ArrayList<>();
    String currentLocation = startPos;

    // Only add paths in sequence - if a destination is None, skip it and all subsequent
    // destinations
    // Actions (intake/shooting) run IN PARALLEL with driving using deadlineWith
    if (dest1 != null && !dest1.equals("None")) {
      Command firstPath = loadPathCommand(alliance, currentLocation, dest1);
      Command firstAction = getActionForDestination(dest1);
      commandSequence.add(firstPath.deadlineWith(firstAction));
      currentLocation = dest1;

      if (dest2 != null && !dest2.equals("None")) {
        Command secondPath = loadPathCommand(alliance, currentLocation, dest2);
        Command secondAction = getActionForDestination(dest2);
        commandSequence.add(secondPath.deadlineWith(secondAction));
        currentLocation = dest2;

        if (dest3 != null && !dest3.equals("None")) {
          Command thirdPath = loadPathCommand(alliance, currentLocation, dest3);
          Command thirdAction = getActionForDestination(dest3);
          commandSequence.add(thirdPath.deadlineWith(thirdAction));
          currentLocation = dest3;
        }
      }
    }

    if (climb) {
      // If the last destination is a depot, use a specific depot-to-tower path first
      // because pathfinding from depot to tower doesn't work well in tight spaces
      if (isDepot(currentLocation)) {
        Command depotToTower = loadDepotToTowerPath(alliance, currentLocation);
        if (depotToTower != null) {
          System.out.println("  Adding depot-to-tower path before climb");
          commandSequence.add(depotToTower);
          // Use ClimbWithoutDrive since we already drove to the tower
          commandSequence.add(ClimbCommands.ClimbWithoutDrive(climberSubsystem));
        } else {
          System.out.println("  WARNING: No depot-to-tower path found for " + currentLocation);
          // Fall back to normal climb with DriveToTower
          try {
            commandSequence.add(ClimbCommands.Climb(drive, climberSubsystem, alliance));
          } catch (Exception e) {
            System.out.println("DynamicAutoBuilder: Climb command not available - skipping");
          }
        }
      } else {
        try {
          commandSequence.add(ClimbCommands.Climb(drive, climberSubsystem, alliance));
        } catch (Exception e) {
          System.out.println("DynamicAutoBuilder: Climb command not available - skipping");
        }
      }
    }

    if (commandSequence.isEmpty()) {
      System.out.println("DynamicAutoBuilder: No paths loaded - running empty auto");
      return Commands.none();
    }

    return Commands.sequence(commandSequence.toArray(new Command[0]));
  }

  /**
   * Returns the path name of the first path in the auto, if any. Used for start-pose lookup (e.g.
   * getAutoStartPose). Empty when there is no first path (climb-only or invalid).
   */
  public Optional<String> getFirstPathName(
      String alliance, String startPos, String dest1, String dest2, String dest3, boolean climb) {
    if (isInvalid(alliance) || isInvalid(startPos)) {
      return Optional.empty();
    }
    if (dest1 == null || dest1.equals("None")) {
      return Optional.empty();
    }
    String name = buildPathName(alliance, startPos, dest1);
    return name != null ? Optional.of(name) : Optional.empty();
  }
}
