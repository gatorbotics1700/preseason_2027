package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Properties;

/** Simple config loader that reads properties files based on roboRIO serial number. */
public final class RobotConfigLoader {
  private static final String SHENANDOAH_SERIAL = "03223852";
  public static final String HUANG_HE_SERIAL = "032D20FA";
  public static final String MISSISSIPPI_SERIAL = "032D2198";
  public static final String NILE_SERIAL = "025AE040";
  private static final String EXPERIMENTAL_SERIAL = "032BE421";
  private static final String DEFAULT_SIM_SERIAL = "SIMULATION";

  private static final String DEFAULT_CONFIG_FILE = "configFiles/config_alecto.properties";
  private static final String SERIAL_PROPERTY_KEY = "robot.serial";
  private static final String SERIAL_ENV_KEY = "ROBOT_SERIAL";
  private static final String CONFIG_PROPERTY_KEY = "robot.config";
  private static final String CONFIG_ENV_KEY = "ROBOT_CONFIG";

  private static Properties config;
  private static String serialOverride;
  private static String configOverride;
  private static String resolvedSerial;

  private RobotConfigLoader() {}

  public static synchronized void setSerialNumberOverride(String overrideSerial) {
    serialOverride = normalize(overrideSerial);
    resolvedSerial = null;
    config = null;
  }

  public static synchronized void setConfigFileOverride(String overrideFile) {
    configOverride = normalize(overrideFile);
    config = null;
  }

  public static synchronized void clearCache() {
    config = null;
    resolvedSerial = null;
  }

  static synchronized Properties load() {
    if (config != null) {
      return config;
    }

    String serial = getSerialNumber();
    String configFile = resolveConfigFile(serial);

    config = new Properties();
    String deployBase = getDeployBasePath();
    try (FileInputStream stream = new FileInputStream(deployBase + "/" + configFile)) {
      config.load(stream);
      System.out.println(
          "RobotConfigLoader: Loaded config '" + configFile + "' (Serial: " + serial + ")");
    } catch (IOException e) {
      throw new RuntimeException("RobotConfigLoader: Failed to load config: " + configFile, e);
    }

    return config;
  }

  /**
   * Returns a deploy directory path that works both on-robot and on desktop tools.
   *
   * <p>On desktop, WPILib's {@link Filesystem#getDeployDirectory()} can require native HAL
   * libraries to be present, which isn't always true for simple command-line tools. In that case,
   * fall back to the project deploy directory.
   */
  private static String getDeployBasePath() {
    try {
      return Filesystem.getDeployDirectory().getPath();
    } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
      // Desktop tool without HAL loaded
    } catch (Throwable t) {
      // Be defensive: if anything goes wrong, fall back to project path.
    }

    Path projectDeploy = Path.of(System.getProperty("user.dir"), "src", "main", "deploy");
    if (Files.isDirectory(projectDeploy)) {
      return projectDeploy.toString();
    }
    return Path.of("src", "main", "deploy").toAbsolutePath().toString();
  }

  public static String getString(String key) {
    String value = load().getProperty(key);
    if (value == null) {
      throw new IllegalArgumentException("Missing config key: " + key);
    }
    return value;
  }

  public static double getDouble(String key) {
    return Double.parseDouble(getString(key));
  }

  public static int getInt(String key) {
    return Integer.parseInt(getString(key));
  }

  public static Boolean getBoolean(String key) {
    return Boolean.parseBoolean(getString(key));
  }

  public static synchronized String getSerialNumber() {
    if (resolvedSerial == null) {
      resolvedSerial = computeSerialNumber();
    }
    return resolvedSerial;
  }

  private static String computeSerialNumber() {
    String fromOverride = normalize(serialOverride);
    if (fromOverride != null) {
      return fromOverride;
    }

    String fromProperty = normalize(System.getProperty(SERIAL_PROPERTY_KEY));
    if (fromProperty != null) {
      return fromProperty;
    }

    String fromEnv = normalize(System.getenv(SERIAL_ENV_KEY));
    if (fromEnv != null) {
      return fromEnv;
    }

    try {
      String serial = normalize(RobotController.getSerialNumber());
      if (serial != null) {
        System.out.println("Read roboRIO serial number " + serial);
        return serial;
      }
    } catch (UnsatisfiedLinkError | NoClassDefFoundError e) {
      // Desktop unit tests (no HAL loaded)
    } catch (Throwable t) {
      System.err.println(
          "RobotConfigLoader: Unable to read roboRIO serial (" + t.getMessage() + ")");
    }

    return DEFAULT_SIM_SERIAL;
  }

  private static String resolveConfigFile(String serial) {
    String fromOverride = normalize(configOverride);
    if (fromOverride != null) {
      return fromOverride;
    }

    String fromProperty = normalize(System.getProperty(CONFIG_PROPERTY_KEY));
    if (fromProperty != null) {
      return fromProperty;
    }

    String fromEnv = normalize(System.getenv(CONFIG_ENV_KEY));
    if (fromEnv != null) {
      return fromEnv;
    }

    return switch (serial) {
      case SHENANDOAH_SERIAL -> "configFiles/config_hulk.properties";
      case HUANG_HE_SERIAL -> "configFiles/config_sting.properties";
      case EXPERIMENTAL_SERIAL -> "configFiles/config_experimental.properties";
      case NILE_SERIAL -> "configFiles/config_alecto.properties";
      default -> {
        System.err.println(
            "RobotConfigLoader: Unknown serial '"
                + serial
                + "', falling back to '"
                + DEFAULT_CONFIG_FILE
                + "'. Set -D"
                + CONFIG_PROPERTY_KEY
                + "=<file> or call setConfigFileOverride() to pick a specific config.");
        yield DEFAULT_CONFIG_FILE;
      }
    };
  }

  private static String normalize(String text) {
    if (text == null) {
      return null;
    }
    String trimmed = text.trim();
    return trimmed.isEmpty() ? null : trimmed;
  }
}
