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

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.RobotConfigLoader;
import java.lang.reflect.Method;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Template for testing the math of getCameraToTargetDistance and getFuelPose in Vision.java.
 *
 * <p>Fill in the TEMPLATE_* values below with a known scenario: robot pose, camera setup, detected
 * target pitch/yaw from PhotonVision, and the theoretically correct outputs. The tests will verify
 * that Vision.java produces those expected values.
 *
 * <p>Set TEMPLATE_IS_FILLED_IN = true once you have entered your values.
 */
class VisionMathTest {

  // =============================================================================
  // TEMPLATE: Fill in these values with your known robot/camera setup and expected outputs
  // =============================================================================

  /** Set to true when you have filled in all template values below. */
  private static final boolean TEMPLATE_IS_FILLED_IN = true;

  /**
   * Config file to load (contains robot-to-camera transforms). e.g.
   * "configFiles/config_sting.properties"
   */
  private static final String TEMPLATE_CONFIG_FILE = "configFiles/config_sting.properties";

  /** Serial number override for config loader. e.g. "03223852" */
  private static final String TEMPLATE_SERIAL = "03223852";

  /** Which camera index to use (0 or 1). Must match a camera in your config. */
  private static final int TEMPLATE_CAMERA_INDEX = 0;

  /**
   * Optional override for getCameraToTargetDistance: camera height (cm). Set to -1 to use config.
   */
  private static final double TEMPLATE_CAMERA_HEIGHT_CM_OVERRIDE = -1;

  /**
   * Optional override for getCameraToTargetDistance: camera pitch (degrees). Set to -999 to use
   * config.
   */
  private static final double TEMPLATE_CAMERA_PITCH_DEG_OVERRIDE = -999;

  /** Robot pose (field coordinates) when the measurement was taken. Units: meters, degrees. */
  private static final double TEMPLATE_ROBOT_POSE_X_METERS = 0;

  private static final double TEMPLATE_ROBOT_POSE_Y_METERS = 0;

  private static final double TEMPLATE_ROBOT_POSE_ROTATION_DEGREES = 0.0;

  /** Target pitch from PhotonVision (degrees). Positive = target above camera center. */
  private static final double TEMPLATE_TARGET_PITCH_DEGREES = -17.6;

  /** Target yaw from PhotonVision (degrees). Positive = target left of camera center. */
  private static final double TEMPLATE_TARGET_YAW_DEGREES = 3.6;

  /**
   * Expected camera-to-target distance from getCameraToTargetDistance. Compute from:
   * (camera_height_cm - 15) / cos(|camera_pitch_rad| + toRadians(target_pitch_deg)) Units:
   * centimeters.
   */
  private static final double TEMPLATE_EXPECTED_DISTANCE_CM = 152;

  /** Expected fuel pose (field coordinates) from getFuelPose. Units: meters, degrees. */
  private static final double TEMPLATE_EXPECTED_FUEL_POSE_X_METERS = -0.12;

  private static final double TEMPLATE_EXPECTED_FUEL_POSE_Y_METERS = -1.81;

  private static final double TEMPLATE_EXPECTED_FUEL_POSE_ROTATION_DEGREES = 90;

  /** Tolerance for distance assertion (cm). Increase if your measurements have uncertainty. */
  private static final double TEMPLATE_DISTANCE_TOLERANCE_CM = 2.0;

  /** Tolerance for pose assertion (m). Increase if your measurements have uncertainty. */
  private static final double TEMPLATE_POSE_TOLERANCE_METERS = 0.1;

  /** Tolerance for fuel pose rotation assertion (degrees). */
  private static final double TEMPLATE_ANGULAR_TOLERANCE_DEGREES = 50;

  // =============================================================================

  @BeforeAll
  static void configureRobotConfigLoader() {
    RobotConfigLoader.setSerialNumberOverride(TEMPLATE_SERIAL);
    RobotConfigLoader.setConfigFileOverride(TEMPLATE_CONFIG_FILE);
    RobotConfigLoader.clearCache();
  }

  @Test
  void testGetCameraToTargetDistance() {
    if (!TEMPLATE_IS_FILLED_IN) {
      fail(
          "Fill in TEMPLATE_* values and set TEMPLATE_IS_FILLED_IN = true to run this test. "
              + "You need: camera height (from config or override), camera pitch (from config or "
              + "override), target pitch, and expected distance (cm).");
    }

    double cameraHeightCm;
    double cameraPitchRad;
    if (TEMPLATE_CAMERA_HEIGHT_CM_OVERRIDE >= 0 && TEMPLATE_CAMERA_PITCH_DEG_OVERRIDE > -999) {
      cameraHeightCm = TEMPLATE_CAMERA_HEIGHT_CM_OVERRIDE;
      cameraPitchRad = Math.toRadians(TEMPLATE_CAMERA_PITCH_DEG_OVERRIDE);
    } else {
      Transform3d robotToCamera =
          VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS_ARRAY[TEMPLATE_CAMERA_INDEX];
      cameraHeightCm = robotToCamera.getMeasureZ().in(Centimeters);
      cameraPitchRad = robotToCamera.getRotation().getY();
    }

    Distance result =
        invokeGetCameraToTargetDistance(
            Math.toRadians(-TEMPLATE_TARGET_PITCH_DEGREES),
            cameraPitchRad,
            Math.toRadians(TEMPLATE_TARGET_YAW_DEGREES),
            cameraHeightCm);
    System.out.println("actual distance from camera to target:" + result.in(Centimeters));
    assertEquals(
        TEMPLATE_EXPECTED_DISTANCE_CM,
        result.in(Centimeters),
        TEMPLATE_DISTANCE_TOLERANCE_CM,
        "getCameraToTargetDistance should return expected distance");
  }

  @Test
  void testGetFuelPose() {
    if (!TEMPLATE_IS_FILLED_IN) {
      fail(
          "Fill in TEMPLATE_* values and set TEMPLATE_IS_FILLED_IN = true to run this test. "
              + "You need: robot pose, target pitch/yaw from PhotonVision, and expected fuel pose.");
    }

    // Replace config-derived camera transform with hardcoded template values so getFuelPose
    // uses the same camera height/pitch as testGetCameraToTargetDistance (Vision reads the
    // array inside getFuelPose, so we must mutate it before calling getFuelPoseFromMockedVision).
    Transform3d originalCameraTransform = null;
    if (TEMPLATE_CAMERA_HEIGHT_CM_OVERRIDE >= 0 && TEMPLATE_CAMERA_PITCH_DEG_OVERRIDE > -999) {
      originalCameraTransform =
          VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS_ARRAY[TEMPLATE_CAMERA_INDEX];
      Translation3d originalCameraTranslation = originalCameraTransform.getTranslation();
      Rotation3d originalCameraRotation = originalCameraTransform.getRotation();
      VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS_ARRAY[TEMPLATE_CAMERA_INDEX] =
          new Transform3d(
              new Translation3d(
                  originalCameraTranslation.getX(),
                  originalCameraTranslation.getY(),
                  TEMPLATE_CAMERA_HEIGHT_CM_OVERRIDE / 100.0),
              new Rotation3d(
                  originalCameraRotation.getX(),
                  Math.toRadians(TEMPLATE_CAMERA_PITCH_DEG_OVERRIDE),
                  originalCameraRotation.getZ()));
    }
    try {
      Pose2d robotPose =
          new Pose2d(
              TEMPLATE_ROBOT_POSE_X_METERS,
              TEMPLATE_ROBOT_POSE_Y_METERS,
              new Rotation2d(Math.toRadians(TEMPLATE_ROBOT_POSE_ROTATION_DEGREES)));

      Pose2d expectedFuelPose =
          new Pose2d(
              TEMPLATE_EXPECTED_FUEL_POSE_X_METERS,
              TEMPLATE_EXPECTED_FUEL_POSE_Y_METERS,
              Rotation2d.fromDegrees(TEMPLATE_EXPECTED_FUEL_POSE_ROTATION_DEGREES));

      Pose2d actual =
          getFuelPoseFromMockedVision(
              robotPose, TEMPLATE_TARGET_PITCH_DEGREES, TEMPLATE_TARGET_YAW_DEGREES);

      System.out.println("actual calculated pose:" + actual);
      assertEquals(
          expectedFuelPose.getX(), actual.getX(), TEMPLATE_POSE_TOLERANCE_METERS, "Fuel pose X");
      assertEquals(
          expectedFuelPose.getY(), actual.getY(), TEMPLATE_POSE_TOLERANCE_METERS, "Fuel pose Y");
      assertEquals(
          expectedFuelPose.getRotation().getRadians(),
          actual.getRotation().getRadians(),
          Math.toRadians(TEMPLATE_ANGULAR_TOLERANCE_DEGREES),
          "Fuel pose rotation");
    } finally {
      if (originalCameraTransform != null) {
        VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS_ARRAY[TEMPLATE_CAMERA_INDEX] =
            originalCameraTransform;
      }
    }
  }

  /** Invokes private getCameraToTargetDistance via reflection. */
  private Distance invokeGetCameraToTargetDistance(
      double pitchInDegrees,
      double cameraAngleInRadians,
      double yawInDegrees,
      double heightInCentimeters) {
    try {
      Method method =
          Vision.class.getDeclaredMethod(
              "getCameraToTargetDistance", double.class, double.class, double.class, double.class);
      method.setAccessible(true);

      Vision vision = new Vision((pose, timestamp, stdDevs) -> {}, mock(VisionIO.class));
      return (Distance)
          method.invoke(
              vision, pitchInDegrees, cameraAngleInRadians, yawInDegrees, heightInCentimeters);
    } catch (Exception e) {
      throw new AssertionError("Failed to invoke getCameraToTargetDistance", e);
    }
  }

  /** Calls Vision.getFuelPose with mocked target. Uses VisionConstants from config. */
  private Pose2d getFuelPoseFromMockedVision(
      Pose2d robotPose, double pitchDegrees, double yawDegrees) {
    VisionIO cameraUnderTest = createMockedVisionIO(pitchDegrees, yawDegrees);

    if (TEMPLATE_CAMERA_INDEX == 0) {
      return new Vision((pose, timestamp, stdDevs) -> {}, cameraUnderTest).getFuelPose(robotPose);
    }
    // For camera 1: camera 0 must not have a fuel target (or lower area) so our target wins
    VisionIO camera0 = createMockedVisionIO_NoFuel();
    return new Vision((pose, timestamp, stdDevs) -> {}, camera0, cameraUnderTest)
        .getFuelPose(robotPose);
  }

  private VisionIO createMockedVisionIO(double pitchDegrees, double yawDegrees) {
    PhotonTrackedTarget target = mock(PhotonTrackedTarget.class);
    when(target.getDetectedObjectClassID()).thenReturn(VisionConstants.FUEL_CLASS_ID);
    when(target.getArea()).thenReturn(1.0);
    when(target.getPitch()).thenReturn(pitchDegrees);
    when(target.getYaw()).thenReturn(yawDegrees);

    PhotonPipelineResult result = mock(PhotonPipelineResult.class);
    when(result.getBestTarget()).thenReturn(target);
    when(result.hasTargets()).thenReturn(true);

    PhotonCamera camera = mock(PhotonCamera.class);
    when(camera.getLatestResult()).thenReturn(result);

    VisionIO visionIO = mock(VisionIO.class);
    when(visionIO.getCamera()).thenReturn(camera);
    return visionIO;
  }

  private VisionIO createMockedVisionIO_NoFuel() {
    PhotonTrackedTarget target = mock(PhotonTrackedTarget.class);
    when(target.getDetectedObjectClassID()).thenReturn(-1); // Not fuel
    when(target.getArea()).thenReturn(0.5);

    PhotonPipelineResult result = mock(PhotonPipelineResult.class);
    when(result.getBestTarget()).thenReturn(target);

    PhotonCamera camera = mock(PhotonCamera.class);
    when(camera.getLatestResult()).thenReturn(result);

    VisionIO visionIO = mock(VisionIO.class);
    when(visionIO.getCamera()).thenReturn(camera);
    return visionIO;
  }
}
