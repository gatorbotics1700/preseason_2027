package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class PointAtHubCommand extends Command {
  private static final int REQUIRED_ON_TARGET_CYCLES = 5;
  private final Drive drive;
  private int onTargetCycles;

  public PointAtHubCommand(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    onTargetCycles = 0;
  }

  @Override
  public void execute() {
    Rotation2d desiredHubAngle = drive.getDesiredHubAngle();
    if (desiredHubAngle == null) {
      onTargetCycles = 0;
      drive.stop();
      return;
    }

    drive.driveToPose(new Pose2d(drive.getPose().getTranslation(), desiredHubAngle));
    Logger.recordOutput("PointAtHub/current angle", drive.getPose().getRotation().getDegrees());
    Logger.recordOutput("PointAtHub/desired angle", desiredHubAngle.getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    Rotation2d desiredHubAngle = drive.getDesiredHubAngle();
    if (desiredHubAngle == null) {
      return false;
    }

    boolean atDesiredRotation =
        drive.calculateRotationError(
                drive.getPose().getRotation().getDegrees(), desiredHubAngle.getDegrees())
            == 0.0;

    onTargetCycles = atDesiredRotation ? onTargetCycles + 1 : 0;
    return onTargetCycles >= REQUIRED_ON_TARGET_CYCLES;
  }
}
