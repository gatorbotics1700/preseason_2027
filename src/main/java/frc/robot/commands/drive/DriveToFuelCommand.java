package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveToFuelConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Calculations;
import org.littletonrobotics.junction.Logger;

public class DriveToFuelCommand extends Command {
  // include any subsystem requirements here
  private final Drive drive;
  private final Vision vision;

  private Pose2d desiredPose;
  private double validTargetSeenTime;

  // constructor
  public DriveToFuelCommand(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    validTargetSeenTime = System.currentTimeMillis();
    addRequirements(drive, vision);
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
    } else {
      // if we no longer see fuel and it's far enough away, we can safely assume somebody else stole
      // it so we should stop trying to go there
      if (desiredPose != null
          && Calculations.distanceToPoseInMeters(currentPose, desiredPose)
              > DriveToFuelConstants.BLIND_SPOT_DEADBAND) {
        desiredPose = null;
      }
    }
    boolean atDesiredPose = false;

    if (desiredPose != null) {
      double xError = drive.calculateDistanceError(currentPose.getX(), desiredPose.getX());
      double yError = drive.calculateDistanceError(currentPose.getY(), desiredPose.getY());
      atDesiredPose = xError == 0.0 && yError == 0.0;
      if (Constants.currentMode == Constants.Mode.SIM) {
        if (atDesiredPose) {
          vision.deleteClosestSimulatedTarget(currentPose);
        }
      }
    }

    double timeSinceValidTargetSeen = System.currentTimeMillis() - validTargetSeenTime;
    if (atDesiredPose
        || desiredPose == null
            && timeSinceValidTargetSeen > DriveToFuelConstants.MAX_IDLE_MILLISECONDS) {
      drive.runVelocity(
          new ChassisSpeeds(0, 0, DriveToFuelConstants.ROTATING_SPEED_RADIANS_PER_SECOND));
    } else if (desiredPose != null) {
      drive.driveToPose(desiredPose);
      validTargetSeenTime = System.currentTimeMillis();
    } else {
      drive.stop();
    }
    Logger.recordOutput("DriveToFuel/Desired Pose", desiredPose);
    Logger.recordOutput("DriveToFuel/Current Pose", currentPose);
    Logger.recordOutput("DriveToFuel/Time Since Target Seen", timeSinceValidTargetSeen);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
