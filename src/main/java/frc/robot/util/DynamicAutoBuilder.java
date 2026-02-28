package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.commands.mech.ClimbCommands;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.commands.mech.ShootingCommand;
import frc.robot.commands.mech.TurretHomingCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.ClimberSubsystem;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class DynamicAutoBuilder {

  private final IntakeSubsystem intakeSubsystem;
  private final Drive drive;
  private final ClimberSubsystem climberSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final Supplier<Pose2d> robotPose;
  private final Supplier<ChassisSpeeds> chassisSpeeds;

  public DynamicAutoBuilder(
      IntakeSubsystem intakeSubsystem,
      Drive drive,
      ClimberSubsystem climberSubsystem,
      HoodSubsystem hoodSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    this.intakeSubsystem = intakeSubsystem;
    this.drive = drive;
    this.climberSubsystem = climberSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.robotPose = robotPose;
    this.chassisSpeeds = chassisSpeeds;
  }

  private String convertFromLocation(String location) {
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

  private boolean isInAllianceZone() {
    double x = robotPose.get().getX();
    return x <= FieldCoordinates.BLUE_BUMP_AND_TRENCH_X
        || x >= FieldCoordinates.RED_BUMP_AND_TRENCH_X;
  }

  /** Creates shooting command that shoots only when in alliance zone. */
  private Command createShootingWithZoneCheck() {
    return Commands.run(() -> shooterSubsystem.setShouldShoot(isInAllianceZone()))
        .alongWith(
            new ShootingCommand(
                shooterSubsystem,
                hoodSubsystem,
                turretSubsystem,
                hopperFloorSubsystem,
                robotPose,
                chassisSpeeds));
  }

  /**
   * Creates homing command for turret and hood at auto start. Skips in sim since sensors don't
   * work.
   */

  // just call HomeMechanisms
  private Command createHomingCommand() {
    if (RobotBase.isSimulation()) {
      System.out.println("  Skipping homing commands in simulation");
      return Commands.none();
    }
    return new TurretHomingCommand(turretSubsystem).alongWith(HoodCommands.HomeHood(hoodSubsystem));
  }

  private Command getActionForDestination(String destination) {
    if (destination == null || destination.equals("None")) {
      return Commands.none();
    }

    if (destination.startsWith("Fuel Pile") && RobotBase.isReal()) {
      return new HoodCommands.HoodRetractCommand(hoodSubsystem)
          .onlyWhile(() -> !isInAllianceZone());
    }

    return Commands.none();
  }

  private Command loadPathCommand(String alliance, String from, String to) {
    String pathName = buildPathName(alliance, from, to);
    if (pathName == null) {
      System.out.println("  Could not build path name for: " + alliance + " " + from + " to " + to);
      return Commands.none();
    }

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      System.out.println("  Loaded path: " + pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      System.out.println("  Path not available: " + pathName + " - " + e.getMessage());
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

    // Build the path sequence (without climb)
    List<Command> pathSequence = new ArrayList<>();

    if (dest1 != null && !dest1.equals("None")) {
      Command firstPath = loadPathCommand(alliance, currentLocation, dest1);
      Command firstAction = getActionForDestination(dest1);
      pathSequence.add(firstPath.deadlineFor(firstAction));
      currentLocation = dest1;

      if (dest2 != null && !dest2.equals("None")) {
        Command secondPath = loadPathCommand(alliance, currentLocation, dest2);
        Command secondAction = getActionForDestination(dest2);
        pathSequence.add(secondPath.deadlineFor(secondAction));
        currentLocation = dest2;

        if (dest3 != null && !dest3.equals("None")) {
          Command thirdPath = loadPathCommand(alliance, currentLocation, dest3);
          Command thirdAction = getActionForDestination(dest3);
          pathSequence.add(thirdPath.deadlineFor(thirdAction));
          currentLocation = dest3;
        }
      }
    }

    // Start with homing command (turret and hood) - skipped in sim
    commandSequence.add(createHomingCommand());

    // Deploy intake once at start (skip mech in sim)
    if (RobotBase.isReal()) {
      commandSequence.add(IntakeCommands.DeployIntake(intakeSubsystem));
    }

    // Run all paths with intake and shooting running continuously
    // Intake runs all the time, shooting only fires when in alliance zone
    if (!pathSequence.isEmpty()) {
      Command allPaths = Commands.sequence(pathSequence.toArray(new Command[0]));
      if (RobotBase.isReal()) {
        Command intakeAndShooting =
            IntakeCommands.RunIntake(intakeSubsystem).alongWith(createShootingWithZoneCheck());
        // Paths are the deadline - when paths finish, intake/shooting stop (until climb or end)
        commandSequence.add(allPaths.deadlineFor(intakeAndShooting));
      } else {
        // In sim, just run the paths without mech
        commandSequence.add(allPaths);
      }
    }

    if (climb) {

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
            commandSequence.add(ClimbCommands.Climb(drive, climberSubsystem));
          } catch (Exception e) {
            System.out.println("DynamicAutoBuilder: Climb command not available - skipping");
          }
        }
      } else {
        try {
          commandSequence.add(ClimbCommands.Climb(drive, climberSubsystem));
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
