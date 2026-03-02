package frc.robot.util;

/**
 * Serialized form of the shot lookup table.
 *
 * <p>This is intentionally primitives-only so it can be written/read quickly and without needing
 * custom JSON adapters for WPILib geometry types.
 */
public class ShotTableData {
  public int schemaVersion;
  public long generatedAtEpochMs;
  public String configHash;

  public double elevationMeters;

  public int tangentialVeloIncrements;
  public int radialVeloIncrements;
  public int rangeIncrements;

  public double maxComponentVelo;
  public double veloIncrement;
  public double maxRange;
  public double rangeIncrement;

  /**
   * Flattened arrays, length = tangentialVeloIncrements * radialVeloIncrements * rangeIncrements.
   */
  public double[] turretAdjustRad;

  /**
   * Flattened arrays, length = tangentialVeloIncrements * radialVeloIncrements * rangeIncrements.
   */
  public double[] hoodAngleRad;

  /**
   * Flattened arrays, length = tangentialVeloIncrements * radialVeloIncrements * rangeIncrements.
   */
  public double[] shotSpeedMps;

  public int size() {
    return tangentialVeloIncrements * radialVeloIncrements * rangeIncrements;
  }
}
