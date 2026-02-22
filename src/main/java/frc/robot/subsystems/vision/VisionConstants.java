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

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.RobotConfigLoader;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout APRIL_TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static final String CAMERA_0_NAME = RobotConfigLoader.getString("camera.0.name");

  public static final double ROBOT_TO_CAMERA_0_X_METERS =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.x_meters");
  public static final double ROBOT_TO_CAMERA_0_Y_METERS =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.y_meters");
  public static final double ROBOT_TO_CAMERA_0_Z_METERS =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.z_meters");
  public static final double CAMERA_0_ROLL_DEGREES =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.roll_degrees");
  public static final double CAMERA_0_PITCH_DEGREES =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.pitch_degrees");
  public static final double ROBOT_TO_CAMERA_0_YAW_DEGREES =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_0.yaw_degrees");

  public static Transform3d ROBOT_TO_CAMERA_0 = createRobotToCamera0Transform();

  // Camera names, must match names configured on coprocessor
  public static final String CAMERA_1_NAME = RobotConfigLoader.getString("camera.1.name");

  public static final double ROBOT_TO_CAMERA_1_X_METERS =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.x_meters");
  public static final double ROBOT_TO_CAMERA_1_Y_METERS =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.y_meters");
  public static final double ROBOT_TO_CAMERA_1_Z_METERS =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.z_meters");
  public static final double ROBOT_TO_CAMERA_1_ROLL_DEGREES =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.roll_degrees");
  public static final double ROBOT_TO_CAMERA_1_PITCH_DEGREES =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.pitch_degrees");
  public static final double ROBOT_TO_CAMERA_1_YAW_DEGREES =
      RobotConfigLoader.getDouble("photonvision.robot_to_camera_1.yaw_degrees");

  public static Transform3d ROBOT_TO_CAMERA_1 = createRobotToCamera1Transform();

  public static Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS_ARRAY = createCameraTransformsArray();

  // Basic filtering thresholds
  public static double MAX_AMBIGUITY = RobotConfigLoader.getDouble("photonvision.max_ambiguity");
  public static double MAX_Z_ERROR = RobotConfigLoader.getDouble("photonvision.max_z_error");

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double LINEAR_STD_DEV_BASELINE =
      RobotConfigLoader.getDouble("photonvision.linear_std_dev_baseline"); // Meters
  public static double ANGULAR_STD_DEV_BASELINE =
      RobotConfigLoader.getDouble("photonvision.angular_std_dev_baseline"); // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)

  public static double CAMERA_0_STD_DEV_FACTOR =
      RobotConfigLoader.getDouble("photonvision.camera0_std_dev_factor");

  public static double CAMERA_1_STD_DEV_FACTOR =
      RobotConfigLoader.getDouble("photonvision.camera1_std_dev_factor");

  public static double[] CAMERA_STD_DEV_FACTORS = createCameraStdDevFactors();

  // Multipliers to apply for PhotonVision multitag observations
  public static double LINEAR_STD_DEV_MULTITAG_FACTOR =
      RobotConfigLoader.getDouble(
          "photonvision.linear_std_dev_multitag_factor"); // More stable than single tag
  public static double ANGULAR_STD_DEV_MULTITAG_FACTOR =
      RobotConfigLoader.getDouble("photonvision.angular_std_dev_multitag_factor");

  public static int FUEL_CLASS_ID = 0;

  public static Transform3d createRobotToCamera0Transform() {
    return createRobotToCameraTransform(
        ROBOT_TO_CAMERA_0_X_METERS,
        ROBOT_TO_CAMERA_0_Y_METERS,
        ROBOT_TO_CAMERA_0_Z_METERS,
        CAMERA_0_ROLL_DEGREES,
        CAMERA_0_PITCH_DEGREES,
        ROBOT_TO_CAMERA_0_YAW_DEGREES);
  }

  public static Transform3d createRobotToCamera1Transform() {
    return createRobotToCameraTransform(
        ROBOT_TO_CAMERA_1_X_METERS,
        ROBOT_TO_CAMERA_1_Y_METERS,
        ROBOT_TO_CAMERA_1_Z_METERS,
        ROBOT_TO_CAMERA_1_ROLL_DEGREES,
        ROBOT_TO_CAMERA_1_PITCH_DEGREES,
        ROBOT_TO_CAMERA_1_YAW_DEGREES);
  }

  public static Transform3d createRobotToCameraTransform(
      double xMeters,
      double yMeters,
      double zMeters,
      double rollDegrees,
      double pitchDegrees,
      double yawDegrees) {
    return new Transform3d(
        xMeters,
        yMeters,
        zMeters,
        new Rotation3d(
            Math.toRadians(rollDegrees), Math.toRadians(pitchDegrees), Math.toRadians(yawDegrees)));
  }

  public static Transform3d[] createCameraTransformsArray() {
    Transform3d[] array = {ROBOT_TO_CAMERA_0, ROBOT_TO_CAMERA_1};
    return array;
  }

  /** Creates array of camera std dev factors from config values. */
  public static double[]
      createCameraStdDevFactors() { // can add more constants if we have more cameras
    return new double[] {CAMERA_0_STD_DEV_FACTOR, CAMERA_1_STD_DEV_FACTOR};
  }

  public static final double DISTANCE_DEADBAND_METERS = 0.03;
  public static final double ROTATION_DEADBAND_DEGREES = 10;
}
