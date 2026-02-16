// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.RobotConfigLoader;

/**
 * This class defines the runtime mode used by AdvantageKit and loads robot-specific configuration
 * based on the roboRIO serial number. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a
 * file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Robot identification
  public static final String ROBOT_SERIAL_NUMBER;

  // Vision Constants (loaded from config)

  static {
    // Load configuration based on roboRIO serial number (auto-loads on first access)
    ROBOT_SERIAL_NUMBER = RobotConfigLoader.getSerialNumber();
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Translation2d FIELD_CENTER = new Translation2d(8.270494, 4.034663);
  public static final double BLUE_BUMP_AND_TRENCH_X = 4.626;
  public static final double RED_BUMP_AND_TRENCH_X = 11.915;

  public static final Distance CENTER_TO_BUMPER_OFFSET = Centimeters.of(40);
  // left and right offsets for the poles on the reef
  public static final Distance CENTER_TO_POLE_OFFSET = Centimeters.of(16.5);
  public static final Distance ROBOT_RADIUS_WITH_BUMPERS = Centimeters.of(57);

  /* MECH */
  public static final int INTAKE_MOTOR_CAN_ID = 11; // needs changing
  public static final int EXTENSION_MOTOR_CAN_ID = 12; // needs changing
  public static final int FLYWHEEL_MOTOR_CAN_ID = 30;
  public static final int HOOD_MOTOR_CAN_ID = 17;
  public static final int KICKER_MOTOR_LOW_CAN_ID = 15; // this and all below need changing
  public static final int KICKER_MOTOR_HIGH_CAN_ID = 31;
  public static final int HOPPER_MOTOR_CAN_ID = 16;
  public static final int TURRET_MOTOR_CAN_ID = 14;
  public static final int INTAKE_DEPLOY_MOTOR_CAN_ID = 9;
  public static final int CLIMBER_MOTOR_CAN_ID = 36;

  public static final int KRAKEN_TICKS_PER_REV = 2048;
  public static final double FLYWHEEL_RADIUS_METERS = 0.0508;

  public static final double TURRET_DEADBAND = 0.75;

  public static final int LOW_RUNG_ARM_LENGTH =
      27; // these are the heights of the rungs from the floor, the inches we want the arm to extend
  // will likely differ
  public static final int MID_RUNG_ARM_LENGTH = 18;
  public static final int HIGH_RUNG_ARM_LENGTH = 18;
  public static final double HOPPER_FLOOR_SPEED = 9; // TODO find a real number
  public static final Translation3d BOT_TO_SHOOTER =
      new Translation3d(
          0.146, 0,
          0.368); // TODO figure out what part of the shooter to measure from (this is the center of
  // the turret plate)
  public static final Translation3d BLUE_HUB =
      new Translation3d(
          4.625594, 4.034663,
          1.80); // z value is 2 centimeters below the very top of the hub (to make sure we aren't
  // trying to phase through walls)
  // hub
}
