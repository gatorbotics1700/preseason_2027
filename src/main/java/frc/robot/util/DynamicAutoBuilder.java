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

public class DynamicAutoBuilder {

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
    if (location.startsWith("Fuel Pile ") && location.length() == 12) {
      return location.substring(0, 11);
    }
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

  public Command buildAuto(
      String alliance, String startPos, String dest1, String dest2, String dest3, boolean climb) {

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

    if (dest1 != null && !dest1.equals("None")) {
      Command firstPath = loadPathCommand(alliance, currentLocation, dest1);
      commandSequence.add(firstPath);
      commandSequence.add(getActionForDestination(dest1));
      currentLocation = dest1;
    }

    if (dest2 != null && !dest2.equals("None")) {
      Command secondPath = loadPathCommand(alliance, currentLocation, dest2);
      commandSequence.add(secondPath);
      commandSequence.add(getActionForDestination(dest2));
      currentLocation = dest2;
    }

    if (dest3 != null && !dest3.equals("None")) {
      Command thirdPath = loadPathCommand(alliance, currentLocation, dest3);
      commandSequence.add(thirdPath);
      commandSequence.add(getActionForDestination(dest3));
      currentLocation = dest3;
    }

    if (climb) {
      try {
        commandSequence.add(ClimbCommands.Climb(drive, climberSubsystem));
      } catch (Exception e) {
        System.out.println("DynamicAutoBuilder: Climb command not available - skipping");
      }
    }

    if (commandSequence.isEmpty()) {
      System.out.println("DynamicAutoBuilder: No paths loaded - running empty auto");
      return Commands.none();
    }

    return Commands.sequence(commandSequence.toArray(new Command[0]));
  }
}
