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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.Calculations;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final List<Pose3d> tagPoses = new LinkedList<>();

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @AutoLogOutput(key = "Odometry/Fuel")
  public Pose2d getFuelPose(Pose2d robotPose) {
    Pose3d robotPose3d = new Pose3d(robotPose);
    Pose2d fuelPose = null;
    double maxArea = 0;
    for (int cameraIndex = 0; cameraIndex < 1; cameraIndex++) { // TODO: fix this for loop range
      PhotonPipelineResult result = io[cameraIndex].getCamera().getLatestResult();
      PhotonTrackedTarget target = result.getBestTarget();
      if (target != null && target.getDetectedObjectClassID() == VisionConstants.FUEL_CLASS_ID) {
        if (target.getArea() > maxArea) {
          maxArea = target.getArea();

          Transform3d robotToCamera = VisionConstants.ROBOT_TO_CAMERA_TRANSFORMS_ARRAY[cameraIndex];
          System.out.println(
              "robot camera transform: "
                  + robotToCamera
                  + " "
                  + Math.toDegrees(robotToCamera.getRotation().getX())
                  + " "
                  + robotToCamera.getRotation().getMeasureY());
          double cameraPitchRadians =
              VisionConstants.UNANGLED_CAMERA_SPACE_PITCH_ARRAY[cameraIndex];
          // photonvision gives us pitch in degrees, the rotation3d in robotToCamera gives us pitch
          // in radians
          Distance cameraToTargetDistance =
              getCameraToTargetDistance(
                  Math.toRadians(-target.getPitch()),
                  cameraPitchRadians,
                  Math.toRadians(target.getYaw()),
                  robotToCamera.getMeasureZ().in(Centimeters));

          System.out.println(
              "testing revolutionary math 2 "
                  + -target.getPitch()
                  + " "
                  + cameraPitchRadians
                  + " "
                  + cameraToTargetDistance);
          // Pose3d cameraToFuel =
          //     new Pose3d()
          //         .transformBy(
          //             new Transform3d(
          //                 new Translation3d(),
          //                 new Rotation3d(0, Math.toRadians(-target.getPitch()),
          // Math.toRadians(target.getYaw()))))
          //         .transformBy(
          //             new Transform3d(
          //                 new Translation3d(
          //                     cameraToTargetDistance, Centimeters.of(0), Centimeters.of(0)),
          //                 new Rotation3d()));
          // System.out.println(
          //     "****HELLO printing cameratofuel pose "
          //         + cameraToTargetDistance
          //         + " "
          //         + cameraToFuel);
          fuelPose =
              robotPose3d
                  .transformBy(robotToCamera)
                  .transformBy(
                      new Transform3d(
                          new Translation3d(),
                          new Rotation3d(
                              0,
                              Math.toRadians(-target.getPitch()),
                              Math.toRadians(target.getYaw()))))
                  .transformBy(
                      new Transform3d(
                          new Translation3d(
                              cameraToTargetDistance, Centimeters.of(0), Centimeters.of(0)),
                          new Rotation3d()))
                  .toPose2d();
        }
      }
    }
    System.out.println("fuelPose before rotation stuff: " + fuelPose);
    if (fuelPose == null) {
      return null;
    }
    double deltaX = fuelPose.getX() - robotPose.getX();
    double deltaY = fuelPose.getY() - robotPose.getY();
    fuelPose =
        new Pose2d(
            fuelPose.getMeasureX(),
            fuelPose.getMeasureY(),
            Calculations.angleToPoint(deltaX, deltaY)
                .rotateBy(
                    new Rotation2d(
                        Math.toRadians(
                            90)))); // TODO: make the 90 a constant based on where the intake is
    // which is 180
    System.out.println("fuelPose before rotation stuff again: " + fuelPose);
    return fuelPose;
  }

  private Distance getCameraToTargetDistance(
      double pitchInRadians,
      double cameraPitchInRadians,
      double yawInRadians,
      double heightInCentimeters) {
    // TODO: this logic assumes that roll of the camera is 0
    // TODO: photonvision pitch is backwards
    double verticalOffset = heightInCentimeters - 7.5;
    double horizontalDistanceToOffset =
        Math.abs(
            heightInCentimeters
                / Math.tan(cameraPitchInRadians + pitchInRadians)
                / Math.cos(yawInRadians));
    double distanceToTarget =
        Math.sqrt(
            verticalOffset * verticalOffset
                + horizontalDistanceToOffset * horizontalDistanceToOffset);
    System.out.println(
        "***hi i'm in the method imma print out some information: "
            + verticalOffset
            + " "
            + horizontalDistanceToOffset
            + " "
            + heightInCentimeters);
    return Centimeters.of(distanceToTarget);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      this.tagPoses.clear();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = VisionConstants.APRIL_TAG_LAYOUT.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > VisionConstants.MAX_AMBIGUITY) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > VisionConstants.MAX_Z_ERROR // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > VisionConstants.APRIL_TAG_LAYOUT.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > VisionConstants.APRIL_TAG_LAYOUT.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = VisionConstants.LINEAR_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = VisionConstants.ANGULAR_STD_DEV_BASELINE * stdDevFactor;
        // Apply multitag factors for MEGATAG_2 (Limelight) or PHOTONVISION with multiple tags
        if (observation.type() == PoseObservationType.MEGATAG_2
            || (observation.type() == PoseObservationType.PHOTONVISION
                && observation.tagCount() > 1)) {
          linearStdDev *= VisionConstants.LINEAR_STD_DEV_MULTITAG_FACTOR;
          angularStdDev *= VisionConstants.ANGULAR_STD_DEV_MULTITAG_FACTOR;
        }
        if (cameraIndex < VisionConstants.CAMERA_STD_DEV_FACTORS.length) {
          linearStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
          angularStdDev *= VisionConstants.CAMERA_STD_DEV_FACTORS[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public Pose3d getTagPose(int tagid) {
    return this.tagPoses.get(tagid);
  }

  public Pose2d getLineupPose(int tagId, boolean isLeftPipe) {
    AprilTagFieldLayout layout = VisionConstants.APRIL_TAG_LAYOUT;
    Pose3d tagPose = layout.getTagPose(tagId).get();
    Distance lineupXOffset = Centimeters.of(45.72);
    Distance lineupYOffset = Centimeters.of(-10);
    if (isLeftPipe) {
      lineupYOffset = Centimeters.of(10);
    }
    Transform3d lineup =
        new Transform3d(lineupXOffset, lineupYOffset, Centimeters.of(0.0), new Rotation3d());
    Pose2d fieldRelativePose =
        tagPose
            .transformBy(lineup)
            .toPose2d()
            .transformBy(new Transform2d(0, 0, new Rotation2d(Degrees.of(90))));
    return fieldRelativePose;
  }

  public void takePicture() {
    System.out.println("taking picture for camera ");
    io[0].getCamera().takeInputSnapshot();
  }
}
