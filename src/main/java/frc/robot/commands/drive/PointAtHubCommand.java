package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class PointAtHubCommand extends Command {
  private final Drive drive;

  public PointAtHubCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drive.driveToPose(new Pose2d(drive.getPose().getTranslation(), drive.getDesiredHubAngle()));
  }

  @Override
  public boolean isFinished() {
    boolean atDesiredPose =
        drive.calculateRotationError(
                drive.getPose().getRotation().getDegrees(), drive.getDesiredHubAngle().getDegrees())
            == 0.0;

    if (atDesiredPose) {
      drive.stop();
      return true;
    }
    return false;
  }
}
