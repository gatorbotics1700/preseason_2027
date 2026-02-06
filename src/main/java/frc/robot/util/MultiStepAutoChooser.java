package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.ClimberSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class MultiStepAutoChooser {
  private final LoggedDashboardChooser<String> allianceChooser;
  private final LoggedDashboardChooser<String> startPosChooser;
  private final DynamicAutoBuilder dynamicAutoBuilder;

  // First destination choosers (location type + side)
  private final LoggedDashboardChooser<String> firstDestinationTypeChooser;
  private final LoggedDashboardChooser<String> firstDestinationSideChooser;

  // Second destination choosers (location type + side)
  private final LoggedDashboardChooser<String> secondDestinationTypeChooser;
  private final LoggedDashboardChooser<String> secondDestinationSideChooser;

  // Third destination choosers (location type + side)
  private final LoggedDashboardChooser<String> thirdDestinationTypeChooser;
  private final LoggedDashboardChooser<String> thirdDestinationSideChooser;

  private final LoggedDashboardChooser<Boolean> shouldClimbChooser;

  public MultiStepAutoChooser(
      IntakeSubsystem intakeSubsystem, Drive drive, ClimberSubsystem climberSubsystem) {
    // Create the dynamic auto builder
    this.dynamicAutoBuilder = new DynamicAutoBuilder(intakeSubsystem, drive, climberSubsystem);

    // Initialize choosers
    allianceChooser = new LoggedDashboardChooser<>("Auto/Alliance");
    startPosChooser = new LoggedDashboardChooser<>("Auto/Start Position");

    firstDestinationTypeChooser =
        new LoggedDashboardChooser<>("Auto/First Destination/First Destination Type");
    firstDestinationSideChooser =
        new LoggedDashboardChooser<>("Auto/First Destination/First Destination Side");

    secondDestinationTypeChooser =
        new LoggedDashboardChooser<>("Auto/Second Destination/Second Destination Type");
    secondDestinationSideChooser =
        new LoggedDashboardChooser<>("Auto/Second Destination/Second Destination Side");

    thirdDestinationTypeChooser =
        new LoggedDashboardChooser<>("Auto/Third Destination/Third Destination Type");
    thirdDestinationSideChooser =
        new LoggedDashboardChooser<>("Auto/Third Destination/Third Destination Side");

    shouldClimbChooser = new LoggedDashboardChooser<>("Auto/Climb?");

    // Populate alliance chooser with hardcoded values
    allianceChooser.addDefaultOption(
        "None", "None"); // TODO: default it as red or blue for safety???
    allianceChooser.addOption("Red", "R");
    allianceChooser.addOption("Blue", "B");

    // Populate start position chooser with hardcoded values
    startPosChooser.addDefaultOption("None", "None");
    startPosChooser.addOption("Center", "Center");
    startPosChooser.addOption("Left Far", "LF");
    startPosChooser.addOption("Left Near", "LN");
    startPosChooser.addOption("Right Far", "RF");
    startPosChooser.addOption("Right Near", "RN");

    // Populate first destination type chooser
    firstDestinationTypeChooser.addDefaultOption("None", "None");
    firstDestinationTypeChooser.addOption("Depot", "Depot");
    firstDestinationTypeChooser.addOption("Outpost", "Outpost");
    firstDestinationTypeChooser.addOption("Fuel Pile Near", "Fuel Pile Near");
    firstDestinationTypeChooser.addOption("Fuel Pile Far", "Fuel Pile Far");
    firstDestinationTypeChooser.addOption("Fuel Pile Middle", "Fuel Pile Middle");

    // Populate first destination side chooser
    firstDestinationSideChooser.addDefaultOption("None", "None");
    firstDestinationSideChooser.addOption("Left", "Left");
    firstDestinationSideChooser.addOption("Right", "Right");
    firstDestinationSideChooser.addOption("Center", "Center");

    // Populate second destination type chooser
    secondDestinationTypeChooser.addDefaultOption("None", "None");
    secondDestinationTypeChooser.addOption("Depot", "Depot");
    secondDestinationTypeChooser.addOption("Outpost", "Outpost");
    secondDestinationTypeChooser.addOption("Fuel Pile Near", "Fuel Pile Near");
    secondDestinationTypeChooser.addOption("Fuel Pile Far", "Fuel Pile Far");
    secondDestinationTypeChooser.addOption("Fuel Pile Middle", "Fuel Pile Middle");

    // Populate second destination side chooser
    secondDestinationSideChooser.addDefaultOption("None", "None");
    secondDestinationSideChooser.addOption("Left", "Left");
    secondDestinationSideChooser.addOption("Right", "Right");
    secondDestinationSideChooser.addOption("Center", "Center");

    // Populate third destination type chooser
    thirdDestinationTypeChooser.addDefaultOption("None", "None");
    thirdDestinationTypeChooser.addOption("Depot", "Depot");
    thirdDestinationTypeChooser.addOption("Outpost", "Outpost");
    thirdDestinationTypeChooser.addOption("Fuel Pile Near", "Fuel Pile Near");
    thirdDestinationTypeChooser.addOption("Fuel Pile Far", "Fuel Pile Far");
    thirdDestinationTypeChooser.addOption("Fuel Pile Middle", "Fuel Pile Middle");

    // Populate third destination side chooser
    thirdDestinationSideChooser.addDefaultOption("None", "None");
    thirdDestinationSideChooser.addOption("Left", "Left");
    thirdDestinationSideChooser.addOption("Right", "Right");
    thirdDestinationSideChooser.addOption("Center", "Center");

    // Populate climb chooser
    shouldClimbChooser.addDefaultOption("No", false);
    shouldClimbChooser.addOption("Yes", true);

    // This ensures they publish to NetworkTables
    allianceChooser.get();
    startPosChooser.get();
    firstDestinationTypeChooser.get();
    firstDestinationSideChooser.get();
    secondDestinationTypeChooser.get();
    secondDestinationSideChooser.get();
    thirdDestinationTypeChooser.get();
    thirdDestinationSideChooser.get();
    shouldClimbChooser.get();
  }

  /**
   * Combines destination type and side into a full destination name. Examples: - "Depot" + "Center"
   * = "DC" - "Depot" + "Left" = "DL" - "Fuel Pile Far" + "Center" = "Fuel Pile CF" - "Outpost" +
   * any = "Outpost" (side doesn't matter, can be empty/N/A)
   */
  private String combineDestination(String type, String side) {
    if (type == null || type.equals("None")) {
      return "None";
    }

    if (type.equals("Outpost")) {
      // Outpost doesn't need a side, ignore it even if provided
      return "Outpost";
    }

    if (type.equals("Depot")) {
      if (side == null || side.equals("None")) {
        return "None"; // TODO: do we want to default somewhere else?
      }
      switch (side) {
        case "Center":
          return "DC";
        case "Left":
          return "DL";
        case "Right":
          return "DR";
      }
    }

    if (type.startsWith("Fuel Pile")) {
      if (side == null || side.equals("None")) {
        return "None";
      }
      // Extract distance: "Fuel Pile Near" -> "N", "Fuel Pile Far" -> "F", "Fuel Pile Middle" ->
      // "M"
      String distance = "";
      if (type.contains("Near")) {
        distance = "N";
      } else if (type.contains("Far")) {
        distance = "F";
      } else if (type.contains("Middle")) {
        distance = "M";
      }

      // Map side to letter
      String sideLetter = "";
      switch (side) {
        case "Center":
          sideLetter = "C";
          break;
        case "Left":
          sideLetter = "L";
          break;
        case "Right":
          sideLetter = "R";
          break;
      }

      if (!distance.isEmpty() && !sideLetter.isEmpty()) {
        return "Fuel Pile " + sideLetter + distance;
      }
    }

    return "None";
  }

  // TODO: make sure this follows the right naming conventions we ultimately decide on
  private String buildAutoFileName(
      String alliance, String startPos, String dest1, String dest2, String dest3, boolean climb) {
    StringBuilder fileName = new StringBuilder();

    // Alliance
    if (alliance != null && !alliance.equals("None")) {
      fileName.append(alliance.equals("R") ? "R" : "B");
    } else {
      return null; // Need alliance
    }

    // Start position
    if (startPos != null && !startPos.equals("None")) {
      fileName.append(" ").append(startPos);
    } else {
      return null; // Need start position
    }

    // Require at least one destination or climb; otherwise run no auto
    boolean hasDestination = dest1 != null && !dest1.equals("None");
    if (!hasDestination && !climb) {
      return null;
    }

    // Destinations: only add in order, no gaps (location 2 only if 1 is set, location 3 only if 2
    // is set)
    if (dest1 != null && !dest1.equals("None")) {
      fileName.append("-").append(dest1);

      if (dest2 != null && !dest2.equals("None")) {
        fileName.append("-").append(dest2);

        if (dest3 != null && !dest3.equals("None")) {
          fileName.append("-").append(dest3);
        }
      }
    }

    // Climb
    if (climb) {
      fileName.append("-Tower");
    }

    fileName.append(".auto");
    return fileName.toString();
  }

  public void updateChooserOptions() {

    allianceChooser.get();
    startPosChooser.get();
    firstDestinationTypeChooser.get();
    firstDestinationSideChooser.get();
    secondDestinationTypeChooser.get();
    secondDestinationSideChooser.get();
    thirdDestinationTypeChooser.get();
    thirdDestinationSideChooser.get();
    shouldClimbChooser.get();
  }

  public String getSelectedPathName() {
    String autoFileName = buildAutoFileNameFromChoosers();
    return autoFileName != null ? autoFileName : "None";
  }

  private String buildAutoFileNameFromChoosers() {
    String alliance = allianceChooser.get();
    String startPos = startPosChooser.get();

    String firstDestination =
        combineDestination(firstDestinationTypeChooser.get(), firstDestinationSideChooser.get());
    String secondDestination =
        combineDestination(secondDestinationTypeChooser.get(), secondDestinationSideChooser.get());
    String thirdDestination =
        combineDestination(thirdDestinationTypeChooser.get(), thirdDestinationSideChooser.get());

    Boolean shouldClimb = shouldClimbChooser.get();
    boolean climb = shouldClimb != null && shouldClimb;

    return buildAutoFileName(
        alliance, startPos, firstDestination, secondDestination, thirdDestination, climb);
  }

  /**
   * @return The command to run in autonomous, or Commands.none() if no valid selection
   */
  public Command getAutonomousCommand() {
    // Update chooser options first
    updateChooserOptions();

    // Get all selections
    String alliance = allianceChooser.get();
    String startPos = startPosChooser.get();

    String firstDestination =
        combineDestination(firstDestinationTypeChooser.get(), firstDestinationSideChooser.get());
    String secondDestination =
        combineDestination(secondDestinationTypeChooser.get(), secondDestinationSideChooser.get());
    String thirdDestination =
        combineDestination(thirdDestinationTypeChooser.get(), thirdDestinationSideChooser.get());

    Boolean shouldClimb = shouldClimbChooser.get();
    boolean climb = shouldClimb != null && shouldClimb;

    // Use DynamicAutoBuilder to chain paths together
    return dynamicAutoBuilder.buildAuto(
        alliance, startPos, firstDestination, secondDestination, thirdDestination, climb);
  }
}
