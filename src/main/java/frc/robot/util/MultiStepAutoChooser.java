package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Multi-step auto chooser that allows drivers to select alliance, starting position, and path
 * segments step-by-step. The chooser automatically builds the auto command from standardized path
 * names.
 */
public class MultiStepAutoChooser {
  private final LoggedDashboardChooser<String> allianceChooser;
  private final LoggedDashboardChooser<String> startPosChooser;

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

  public MultiStepAutoChooser() {
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
    allianceChooser.addDefaultOption("None", "None");
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
    secondDestinationTypeChooser.addDefaultOption("None (End)", "None");
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
    thirdDestinationTypeChooser.addDefaultOption("None (End)", "None");
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

    // Force initialization by reading each chooser once
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

  /**
   * Builds the auto filename from the current selections. Format: {Alliance} {StartPos} {Dest1}
   * {Dest2} {Dest3} {Climb}.auto Example: R Center DC DL.auto or B LF Fuel Pile CF Outpost
   * Climb.auto
   */
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

    // Destinations (only add if not None)
    if (dest1 != null && !dest1.equals("None")) {
      fileName.append(" ").append(dest1);
    }

    if (dest2 != null && !dest2.equals("None")) {
      fileName.append(" ").append(dest2);
    }

    if (dest3 != null && !dest3.equals("None")) {
      fileName.append(" ").append(dest3);
    }

    // Climb
    if (climb) {
      fileName.append(" Climb");
    }

    fileName.append(".auto");
    return fileName.toString();
  }

  /**
   * Updates chooser options based on current selections. Should be called periodically. Note: Since
   * LoggedDashboardChooser doesn't support dynamic option clearing, all options are populated
   * upfront. This method is kept for potential future enhancements.
   *
   * <p>Reading the choosers here ensures they stay published to NetworkTables.
   */
  public void updateChooserOptions() {
    // Read choosers to ensure they stay published to NetworkTables
    // This is necessary for LoggedDashboardChooser to maintain NetworkTables entries
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
   * Gets the currently selected auto file name for display. Returns the auto filename based on
   * current selections. This method reads chooser values fresh each time it's called.
   */
  public String getSelectedPathName() {
    String autoFileName = buildAutoFileNameFromChoosers();
    return autoFileName != null ? autoFileName : "None";
  }

  /**
   * Builds the auto filename from the current chooser selections. This is a helper method to avoid
   * code duplication.
   */
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
   * Gets the autonomous command based on the current selections.
   *
   * @return The command to run in autonomous, or Commands.none() if no valid selection
   */
  public Command getAutonomousCommand() {
    try {
      // Update chooser options first
      updateChooserOptions();

      // Build auto filename from selections
      String autoFileName = buildAutoFileNameFromChoosers();

      if (autoFileName == null) {
        System.out.println("Selected Auto: None (missing alliance or start position)");
        return Commands.none();
      }

      // Print selected auto file name to console
      System.out.println("Selected Auto: " + autoFileName);
      System.out.flush();

      // Load the auto file using AutoBuilder
      try {
        Command autoCommand = AutoBuilder.buildAuto(autoFileName);
        if (autoCommand != null) {
          return autoCommand;
        } else {
          System.err.println(
              "MultiStepAutoChooser: Auto file not found or could not be loaded: " + autoFileName);
          return Commands.none();
        }
      } catch (Exception e) {
        System.err.println(
            "MultiStepAutoChooser: Error loading auto file "
                + autoFileName
                + ": "
                + e.getMessage());
        e.printStackTrace();
        return Commands.none();
      }
    } catch (Exception e) {
      System.err.println("MultiStepAutoChooser: Error building auto command: " + e.getMessage());
      e.printStackTrace();
      return Commands.none();
    }
  }
}
