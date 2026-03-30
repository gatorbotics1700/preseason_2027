package frc.robot.util.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotCalculatorConditions;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Properties;

/**
 * Desktop/offboard tool to generate the shot lookup table once and save it into {@code
 * src/main/deploy} so it is copied to the robot on deploy.
 */
public final class ShotTableGenerator {
  private ShotTableGenerator() {}

  public static void main(String[] args) throws Exception {
    String tableName = "AlectoHub";
    String configRel = "configFiles/config_alecto.properties";
    String outRel = "shot_calculator/" + tableName + ".json";
    boolean pretty = false;

    for (int i = 0; i < args.length; i++) {
      switch (args[i]) {
        case "--name" -> {
          if (i + 1 >= args.length) {
            throw new IllegalArgumentException("--name requires a table name");
          }
          tableName = args[++i];
          outRel = "shot_calculator/" + tableName + ".json";
        }
        case "--config" -> {
          if (i + 1 >= args.length) {
            throw new IllegalArgumentException("--config requires a relative path");
          }
          configRel = args[++i];
        }
        case "--out" -> {
          if (i + 1 >= args.length) {
            throw new IllegalArgumentException("--out requires a path");
          }
          outRel = args[++i];
        }
        case "--pretty" -> pretty = true;
        default -> throw new IllegalArgumentException("Unknown arg: " + args[i]);
      }
    }

    // Load Alecto hood settings directly from the properties file (no RobotConfigLoader / no HAL).
    Path configPath = ShotTableIO.projectDeployPath(configRel);
    Properties props = new Properties();
    try (InputStream in = Files.newInputStream(configPath)) {
      props.load(in);
    }
    double hoodRetractedDeg = Double.parseDouble(props.getProperty("mech.hood_retracted_degrees"));
    double hoodMinDeg = Double.parseDouble(props.getProperty("mech.hood_min_angle_degrees"));
    Rotation2d hoodRetracted = Rotation2d.fromDegrees(hoodRetractedDeg);
    Rotation2d hoodMin = Rotation2d.fromDegrees(hoodMinDeg);

    double elevationMeters =
        FieldCoordinates.BLUE_HUB.getZ() - ShooterConstants.BOT_TO_SHOOTER.getZ();
    Path outPath = ShotTableIO.projectDeployPath(outRel);

    if (ShotTableIO.isUpToDate(
        outPath, elevationMeters, hoodRetracted.getRadians(), hoodMin.getRadians())) {
      System.out.println("Shot table already up-to-date at " + outPath);
      return;
    }

    int veloIncrements =
        (int)
            (ShotCalculatorConditions.MAX_COMPONENT_VELO
                * 2
                / ShotCalculatorConditions.VELO_INCREMENT);
    int rangeIncrements =
        (int) (ShotCalculatorConditions.MAX_RANGE / ShotCalculatorConditions.RANGE_INCREMENT);

    System.out.println("Generating shot table...");
    ShotParameters[][][] table =
        ShotCalculator.getShootingLookupTable(elevationMeters, hoodMin, hoodRetracted);
    ShotTableData data =
        ShotTableIO.fromLookupTable(
            table,
            elevationMeters,
            hoodRetracted.getRadians(),
            hoodMin.getRadians(),
            veloIncrements,
            veloIncrements,
            rangeIncrements);

    ShotTableIO.writeJson(outPath, data, pretty);
    System.out.println("Wrote shot table JSON to " + outPath);
  }
}
