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

  public static final int KRAKEN_TICKS_PER_REV = 2048;

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
  public static final int OUTER_ARM_MOTOR_CAN_ID = 18;
  public static final int INNER_ARM_MOTOR_CAN_ID = 19;

  public static final int HOOD_GEAR_RATIO = 9; // for prototype mechanism
  public static final int TURRET_GEAR_RATIO = 50; // TODO: updated by katy and niva 1/24
  public static final int KRAKEN_TICKS_PER_REV = 2048;
  public static final int CLIMBER_TICKS_PER_INCH = 40; // TODO: change!!

  public static final double TURRET_DEADBAND = 0.75;

  public static final int LOW_RUNG_ARM_LENGTH =
      27; // these are the heights of the rungs from the floor, the inches we want the arm to extend
  // will likely differ
  public static final int MID_RUNG_ARM_LENGTH = 18;
  public static final int HIGH_RUNG_ARM_LENGTH = 18;

  public static final double CLIMBER_EXTENDED_POSITION = 10; // change these values
  public static final int CLIMBER_RETRACTED_POSITION = 0;

  public static final double HOOD_SHAFT_REVS_PER_MECH_REV =
      208 / 15; // big gear teeth / little gear teeth
  public static final double FLYWHEEL_SHOOTING_VOLTAGE = 8.9;
  public static final double KICKER_SHOOTING_VOLTAGE = 8;
}
