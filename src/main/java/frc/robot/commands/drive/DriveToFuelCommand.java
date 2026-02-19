package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Calculations;
import org.littletonrobotics.junction.Logger;

public class DriveToFuelCommand extends Command {
  // include any subsystem requirements here
  private final Drive drive;
  private final Vision vision;

  private Pose2d desiredPose;

  private static final double BLIND_SPOT_DEADBAND = 0.5; // TODO change
  private static final double MAX_IDLE_TIME_SECONDS = 2; // TODO change based off real world maybe?

  private static final double ROTATING_SPEED_RADIANS_PER_SECOND =
      2; // TODO change based off real life

  private double validTargetSeenTime;

  // constructor
  public DriveToFuelCommand(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    // run any methods that should only run once (set desired position, etc)
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Pose2d newFuelPose;
    if (Constants.currentMode == Constants.Mode.SIM) {
      newFuelPose = vision.tempGetFuelPoseInSim(currentPose);
    } else {
      newFuelPose = vision.getFuelPose(currentPose);
    }
    if (newFuelPose != null) {
      // TODO think about the case where theres a ball in the blind spot but you still see another
      // target uh oh
      desiredPose = newFuelPose;
      double xError = drive.calculateDistanceError(currentPose.getX(), desiredPose.getX());
      double yError = drive.calculateDistanceError(currentPose.getY(), desiredPose.getY());
      double rotationError =
          drive.calculateRotationError(
              currentPose.getRotation().getDegrees(), desiredPose.getRotation().getDegrees());
      boolean atDesiredPose = xError == 0.0 && yError == 0.0;
      if (atDesiredPose) {
        if (Constants.currentMode == Constants.Mode.SIM) {
          vision.deleteClosestFuel(currentPose);
        }
        desiredPose = null;
      }
    } else {
      // if we no longer see fuel and it's far enough away, we can safely assume somebody else stole
      // it so we should stop trying to go there
      if (desiredPose != null
          && Calculations.distanceToPoseInMeters(currentPose, desiredPose) > BLIND_SPOT_DEADBAND) {
        desiredPose = null;
      }
    }
    Logger.recordOutput("Odometry/Desired Pose in Intake", desiredPose);
    Logger.recordOutput("Odometry/Current Pose in Intake", currentPose);
    System.out.println("this is the desired pose here u go: " + desiredPose);
    if (desiredPose != null) {
      drive.driveToPose(desiredPose);
      validTargetSeenTime = System.currentTimeMillis();
    } else if (System.currentTimeMillis() - validTargetSeenTime > MAX_IDLE_TIME_SECONDS) {
      drive.runVelocity(new ChassisSpeeds(0, 0, ROTATING_SPEED_RADIANS_PER_SECOND));
    } else {
      drive.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
