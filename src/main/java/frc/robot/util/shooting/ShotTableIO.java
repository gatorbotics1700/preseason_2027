package frc.robot.util.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShotCalculatorConditions;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HexFormat;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public final class ShotTableIO {
  private ShotTableIO() {}

  public static final int SCHEMA_VERSION = 1;

  /** Default location inside the WPILib deploy directory. */
  public static final String DEFAULT_DEPLOY_RELATIVE_PATH = "shot_calculator/shot_table.json";

  /**
   * Resolve a path under the project deploy directory ({@code src/main/deploy}).
   *
   * <p>This is intended for desktop/offboard tools. Robot code should resolve under {@code
   * Filesystem.getDeployDirectory()} instead.
   */
  public static Path projectDeployPath(String relativePath) {
    return Path.of("src", "main", "deploy").resolve(relativePath);
  }

  public static String computeConfigHash(
      double elevationMeters, double hoodRetractedRad, double hoodMinRad) {
    // Include any values that materially affect the generated table.
    String config =
        "schema="
            + SCHEMA_VERSION
            + "\n"
            + "elevationMeters="
            + elevationMeters
            + "\n"
            + "SHOT_DEADBAND="
            + ShotCalculatorConditions.SHOT_DEADBAND
            + "\n"
            + "MIN_SHOT_HEIGHT="
            + ShotCalculatorConditions.MIN_SHOT_HEIGHT
            + "\n"
            + "MAX_SHOT_HEIGHT="
            + ShotCalculatorConditions.MAX_SHOT_HEIGHT
            + "\n"
            + "MAX_SHOT_SPEED="
            + String.format("%.2f", ShotCalculatorConditions.MAX_SHOT_SPEED)
            + "\n"
            + "VELO_INCREMENT="
            + ShotCalculatorConditions.VELO_INCREMENT
            + "\n"
            + "RANGE_INCREMENT="
            + ShotCalculatorConditions.RANGE_INCREMENT
            + "\n"
            + "MAX_COMPONENT_VELO="
            + ShotCalculatorConditions.MAX_COMPONENT_VELO
            + "\n"
            + "MAX_RANGE="
            + String.format("%.2f", ShotCalculatorConditions.MAX_RANGE)
            + "\n"
            + "HOOD_RETRACTED_RAD="
            + String.format("%.2f", hoodRetractedRad)
            + "\n"
            + "HOOD_MIN_RAD="
            + String.format("%.2f", hoodMinRad)
            + "\n"
            + "BOT_TO_SHOOTER_X="
            + String.format("%.2f", ShooterConstants.BOT_TO_SHOOTER.getX())
            + "\n"
            + "BOT_TO_SHOOTER_Y="
            + String.format("%.2f", ShooterConstants.BOT_TO_SHOOTER.getY())
            + "\n"
            + "BOT_TO_SHOOTER_Z="
            + String.format("%.2f", ShooterConstants.BOT_TO_SHOOTER.getZ())
            + "\n";

    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      byte[] hash = digest.digest(config.getBytes(StandardCharsets.UTF_8));
      return HexFormat.of().formatHex(hash);
    } catch (NoSuchAlgorithmException e) {
      // Should never happen on a standard JVM.
      throw new RuntimeException(e);
    }
  }

  public static ShotTableData fromLookupTable(
      ShotParameters[][][] table,
      double elevationMeters,
      int tangentialVeloIncrements,
      int radialVeloIncrements,
      int rangeIncrements) {
    ShotTableData data = new ShotTableData();
    data.schemaVersion = SCHEMA_VERSION;
    data.generatedAtEpochMs = System.currentTimeMillis();
    data.elevationMeters = elevationMeters;
    data.tangentialVeloIncrements = tangentialVeloIncrements;
    data.radialVeloIncrements = radialVeloIncrements;
    data.rangeIncrements = rangeIncrements;
    data.maxComponentVelo = ShotCalculatorConditions.MAX_COMPONENT_VELO;
    data.veloIncrement = ShotCalculatorConditions.VELO_INCREMENT;
    data.maxRange = ShotCalculatorConditions.MAX_RANGE;
    data.rangeIncrement = ShotCalculatorConditions.RANGE_INCREMENT;

    int n = data.size();
    data.turretAdjustRad = new double[n];
    data.hoodAngleRad = new double[n];
    data.shotSpeedMps = new double[n];

    int idx = 0;
    for (int i = 0; i < tangentialVeloIncrements; i++) {
      for (int j = 0; j < radialVeloIncrements; j++) {
        for (int k = 0; k < rangeIncrements; k++) {
          ShotParameters p = table[i][j][k];
          data.turretAdjustRad[idx] = p.turretAngle.getRadians();
          data.hoodAngleRad[idx] = p.hoodAngle.getRadians();
          data.shotSpeedMps[idx] = p.shotSpeed;
          idx++;
        }
      }
    }

    // Generator should pass hood angles via the overload below. Robot runtime computes hash on
    // load.
    data.configHash = "";
    return data;
  }

  public static ShotTableData fromLookupTable(
      ShotParameters[][][] table,
      double elevationMeters,
      double hoodRetractedRad,
      double hoodMinRad,
      int tangentialVeloIncrements,
      int radialVeloIncrements,
      int rangeIncrements) {
    ShotTableData data =
        fromLookupTable(
            table,
            elevationMeters,
            tangentialVeloIncrements,
            radialVeloIncrements,
            rangeIncrements);
    data.configHash = computeConfigHash(elevationMeters, hoodRetractedRad, hoodMinRad);
    return data;
  }

  public static ShotParameters[][][] toLookupTable(ShotTableData data) {
    ShotParameters[][][] table =
        new ShotParameters[data.tangentialVeloIncrements][data.radialVeloIncrements]
            [data.rangeIncrements];

    int idx = 0;
    for (int i = 0; i < data.tangentialVeloIncrements; i++) {
      for (int j = 0; j < data.radialVeloIncrements; j++) {
        for (int k = 0; k < data.rangeIncrements; k++) {
          table[i][j][k] =
              new ShotParameters(
                  new Rotation2d(data.turretAdjustRad[idx]),
                  new Rotation2d(data.hoodAngleRad[idx]),
                  data.shotSpeedMps[idx]);
          idx++;
        }
      }
    }
    return table;
  }

  public static void writeJson(Path path, ShotTableData data, boolean pretty) throws IOException {
    Files.createDirectories(path.getParent());

    JSONObject root = new JSONObject();
    root.put("schemaVersion", data.schemaVersion);
    root.put("generatedAtEpochMs", data.generatedAtEpochMs);
    root.put("configHash", data.configHash);
    root.put("elevationMeters", data.elevationMeters);

    JSONObject dims = new JSONObject();
    dims.put("tangentialVeloIncrements", data.tangentialVeloIncrements);
    dims.put("radialVeloIncrements", data.radialVeloIncrements);
    dims.put("rangeIncrements", data.rangeIncrements);
    root.put("dims", dims);

    JSONObject cfg = new JSONObject();
    cfg.put("maxComponentVelo", data.maxComponentVelo);
    cfg.put("veloIncrement", data.veloIncrement);
    cfg.put("maxRange", data.maxRange);
    cfg.put("rangeIncrement", data.rangeIncrement);
    root.put("config", cfg);

    JSONObject arrays = new JSONObject();
    arrays.put("turretAdjustRad", toJsonArray(data.turretAdjustRad));
    arrays.put("hoodAngleRad", toJsonArray(data.hoodAngleRad));
    arrays.put("shotSpeedMps", toJsonArray(data.shotSpeedMps));
    root.put("data", arrays);

    String json = root.toJSONString();
    if (pretty) {
      json = prettyPrint(json);
    }

    try (BufferedWriter writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8)) {
      writer.write(json);
      writer.newLine();
    }
  }

  public static ShotTableData readJson(Path path) throws IOException {
    JSONParser parser = new JSONParser();
    try (BufferedReader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
      JSONObject root = (JSONObject) parser.parse(reader);

      ShotTableData data = new ShotTableData();
      data.schemaVersion = toInt(root.get("schemaVersion"));
      data.generatedAtEpochMs = toLong(root.get("generatedAtEpochMs"));
      data.configHash = (String) root.get("configHash");
      data.elevationMeters = toDouble(root.get("elevationMeters"));

      JSONObject dims = (JSONObject) root.get("dims");
      data.tangentialVeloIncrements = toInt(dims.get("tangentialVeloIncrements"));
      data.radialVeloIncrements = toInt(dims.get("radialVeloIncrements"));
      data.rangeIncrements = toInt(dims.get("rangeIncrements"));

      JSONObject cfg = (JSONObject) root.get("config");
      data.maxComponentVelo = toDouble(cfg.get("maxComponentVelo"));
      data.veloIncrement = toDouble(cfg.get("veloIncrement"));
      data.maxRange = toDouble(cfg.get("maxRange"));
      data.rangeIncrement = toDouble(cfg.get("rangeIncrement"));

      JSONObject arrays = (JSONObject) root.get("data");
      data.turretAdjustRad = toDoubleArray((JSONArray) arrays.get("turretAdjustRad"));
      data.hoodAngleRad = toDoubleArray((JSONArray) arrays.get("hoodAngleRad"));
      data.shotSpeedMps = toDoubleArray((JSONArray) arrays.get("shotSpeedMps"));

      int expected = data.size();
      if (data.turretAdjustRad.length != expected
          || data.hoodAngleRad.length != expected
          || data.shotSpeedMps.length != expected) {
        throw new IOException(
            "Shot table JSON has wrong array lengths (expected "
                + expected
                + ", got "
                + data.turretAdjustRad.length
                + "/"
                + data.hoodAngleRad.length
                + "/"
                + data.shotSpeedMps.length
                + ")");
      }

      return data;
    } catch (ParseException | ClassCastException e) {
      throw new IOException("Failed to parse shot table JSON: " + path, e);
    }
  }

  public static boolean isUpToDate(
      Path path, double elevationMeters, double hoodRetractedRad, double hoodMinRad) {
    if (!Files.exists(path)) {
      return false;
    }
    try {
      ShotTableData existing = readJson(path);
      if (existing.schemaVersion != SCHEMA_VERSION) {
        return false;
      }
      String expectedHash = computeConfigHash(elevationMeters, hoodRetractedRad, hoodMinRad);
      return expectedHash.equals(existing.configHash);
    } catch (IOException e) {
      return false;
    }
  }

  private static JSONArray toJsonArray(double[] values) {
    JSONArray arr = new JSONArray();
    for (double v : values) {
      arr.add(v);
    }
    return arr;
  }

  private static double[] toDoubleArray(JSONArray arr) {
    double[] out = new double[arr.size()];
    for (int i = 0; i < arr.size(); i++) {
      out[i] = toDouble(arr.get(i));
    }
    return out;
  }

  private static int toInt(Object o) {
    if (o instanceof Long l) {
      return Math.toIntExact(l);
    }
    if (o instanceof Number n) {
      return n.intValue();
    }
    return Integer.parseInt(String.valueOf(o));
  }

  private static long toLong(Object o) {
    if (o instanceof Long l) {
      return l;
    }
    if (o instanceof Number n) {
      return n.longValue();
    }
    return Long.parseLong(String.valueOf(o));
  }

  private static double toDouble(Object o) {
    if (o instanceof Double d) {
      return d;
    }
    if (o instanceof Number n) {
      return n.doubleValue();
    }
    return Double.parseDouble(String.valueOf(o));
  }

  /**
   * Very small pretty-printer to make the JSON readable when desired.
   *
   * <p>We avoid adding new JSON dependencies just for formatting.
   */
  private static String prettyPrint(String minifiedJson) {
    StringBuilder out = new StringBuilder(minifiedJson.length() + 1024);
    int indent = 0;
    boolean inString = false;
    for (int i = 0; i < minifiedJson.length(); i++) {
      char c = minifiedJson.charAt(i);
      if (c == '"' && (i == 0 || minifiedJson.charAt(i - 1) != '\\')) {
        inString = !inString;
      }
      if (!inString) {
        switch (c) {
          case '{', '[' -> {
            out.append(c).append('\n');
            indent++;
            appendIndent(out, indent);
            continue;
          }
          case '}', ']' -> {
            out.append('\n');
            indent = Math.max(0, indent - 1);
            appendIndent(out, indent);
            out.append(c);
            continue;
          }
          case ',' -> {
            out.append(c).append('\n');
            appendIndent(out, indent);
            continue;
          }
          case ':' -> {
            out.append(c).append(' ');
            continue;
          }
          default -> {
            // fall through
          }
        }
      }
      out.append(c);
    }
    return out.toString();
  }

  private static void appendIndent(StringBuilder sb, int indent) {
    for (int i = 0; i < indent; i++) {
      sb.append("  ");
    }
  }
}
