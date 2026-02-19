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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.Calculations;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final List<Pose3d> tagPoses = new LinkedList<>();
  private boolean hasTargetInSim;
  private ArrayList<Translation2d> simulatedTargets = new ArrayList<Translation2d>();

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

    hasTargetInSim = false;
    resetSimulatedTargets();
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

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
          Pose3d cameraInFieldSpace =
              robotPose3d.transformBy(
                  robotToCamera); // TODO: log robotpose and camera pose in advantagescope and spin,
          // testing for accuracy
          List<TargetCorner> corners = target.getMinAreaRectCorners();
          double sumX = 0, sumY = 0;
          for (TargetCorner c : corners) {
            sumX += c.x;
            sumY += c.y;
          }
          double targetPixelsX = sumX / 4.0;
          double targetPixelsY = sumY / 4.0;
          // both pitch and yaw are using right hand coordinate system
          double targetPitchDegrees = (targetPixelsY - 240) / 480 * 52.5;
          double targetYawDegrees =
              -(targetPixelsX - 320)
                  / 640
                  * 70; // TODO: lens distortion might ruin this, so make table with real life
          // values for yaw and pitch
          Logger.recordOutput("Odometry/targetPixelsX", targetPixelsX);
          Logger.recordOutput("Odometry/targetPixelsY", targetPixelsY);

          Logger.recordOutput("Odometry/vivien's made up fuel pitch", targetPitchDegrees);
          Logger.recordOutput("Odometry/vivien's made up fuel yaw", targetYawDegrees);

          cameraInFieldSpace =
              cameraInFieldSpace.transformBy(
                  new Transform3d(
                      new Translation3d(),
                      new Rotation3d(
                          0,
                          Math.toRadians(targetPitchDegrees),
                          Math.toRadians(targetYawDegrees))));
          Translation3d towardFuelInRobotSpace =
              cameraInFieldSpace
                  .transformBy(
                      new Transform3d(
                          new Translation3d(
                              Centimeters.of(155), Centimeters.of(0), Centimeters.of(0)),
                          new Rotation3d()))
                  .getTranslation();
          double a =
              towardFuelInRobotSpace.getX()
                  - cameraInFieldSpace.getX(); // TODO: change these names a,b,c
          double b = towardFuelInRobotSpace.getY() - cameraInFieldSpace.getY();
          double c = towardFuelInRobotSpace.getZ() - cameraInFieldSpace.getZ();
          System.out.println("a,b,c: " + a + " " + b + " " + c);
          Distance fuelPoseX = // TODO: change 7.5 to constant
              (Centimeters.of(7.5).minus(cameraInFieldSpace.getMeasureZ()))
                  .div(c)
                  .times(a)
                  .plus(cameraInFieldSpace.getMeasureX());
          Distance fuelPoseY =
              (Centimeters.of(7.5).minus(cameraInFieldSpace.getMeasureZ()))
                  .div(c)
                  .times(b)
                  .plus(cameraInFieldSpace.getMeasureY());
          fuelPose = new Pose2d(fuelPoseX, fuelPoseY, new Rotation2d());
        }
      }
    }
    // System.out.println("fuelPose before rotation stuff: " + fuelPose);
    if (fuelPose == null) {
      return null;
    }
    double deltaX = fuelPose.getX() - robotPose.getX();
    double deltaY = fuelPose.getY() - robotPose.getY();
    // Logger.recordOutput("Odometry/fuel Pose Z", fuelPose.getZ());

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
    // System.out.println("fuelPose before rotation stuff again: " + fuelPose);
    return fuelPose;
  }

  public Pose2d tempGetFuelPoseInSim(Pose2d robotPose) {

    // System.out.println("fuelPose before rotation stuff: " + fuelPose);
    // if (!hasTargetInSim) {
    //   return null;
    // }
    int closestFuelIndex = getClosestFuelIndex(robotPose);
    Pose2d closestFuelPose = null;
    if (closestFuelIndex != -1) {
      closestFuelPose = new Pose2d(simulatedTargets.get(closestFuelIndex), new Rotation2d());
    } else {
      return closestFuelPose;
    }

    double deltaX = closestFuelPose.getX() - robotPose.getX();
    double deltaY = closestFuelPose.getY() - robotPose.getY();
    // Logger.recordOutput("Odometry/fuel Pose Z", fuelPose.getZ());

    closestFuelPose =
        new Pose2d(
            closestFuelPose.getMeasureX(),
            closestFuelPose.getMeasureY(),
            Calculations.angleToPoint(deltaX, deltaY)
                .rotateBy(
                    new Rotation2d(
                        Math.toRadians(
                            90)))); // TODO: make the 90 a constant based on where the intake is
    // which is 180
    // System.out.println("fuelPose before rotation stuff again: " + fuelPose);
    return closestFuelPose;
  }

  private int getClosestFuelIndex(Pose2d robotPose) {
    double minDistInMeters = 16;
    int closestFuelIndex = -1;
    for (int t = 0; t < simulatedTargets.size(); t++) {
      Translation2d target = simulatedTargets.get(t);
      Pose2d targetPose = new Pose2d(target, new Rotation2d());
      // Logger.recordOutput("Odometry/simuatedTarget" + t, targetPose);
      double robotToTargetDist = Calculations.distanceToPoseInMeters(robotPose, targetPose);
      if (robotToTargetDist < minDistInMeters) {
        minDistInMeters = robotToTargetDist;
        closestFuelIndex = t;
      }
    }
    return closestFuelIndex;
  }

  public void deleteClosestFuel(Pose2d robotPose) {
    simulatedTargets.remove(getClosestFuelIndex(robotPose));
  }

  public void resetSimulatedTargets() {
    simulatedTargets.clear();
    simulatedTargets.add(new Translation2d(6.711, 5.71));
    simulatedTargets.add(new Translation2d(8.356, 3.22));
    simulatedTargets.add(new Translation2d(7.331, 0.896));
    simulatedTargets.add(new Translation2d(9.614, 5.085));
    simulatedTargets.add(new Translation2d(9.501, 6.622));
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

    Logger.recordOutput("Odometry/hasTargetInSim", hasTargetInSim);
    for (int t = 0; t < simulatedTargets.size(); t++) {
      Translation2d target = simulatedTargets.get(t);
      Pose2d targetPose = new Pose2d(target, new Rotation2d());
      Logger.recordOutput("Odometry/simuatedTarget" + t, targetPose);
    }
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

  public void toggleSimHasTarget() {
    hasTargetInSim = !hasTargetInSim;
  }
}
