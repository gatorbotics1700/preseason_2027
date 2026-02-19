package frc.robot.commands.mech;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.vision.Vision;

public class IntakeCommands {

  private static final double INTAKING_VOLTAGE =
      10; // TODO get a real number (I just picked my favorite)

  private IntakeCommands() {}

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(true, intakeSubsystem);
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(false, intakeSubsystem);
  }

  public static Command RunIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setIntakeVoltage(INTAKING_VOLTAGE);
        });
  }

  public static Command StopIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setIntakeVoltage(0);
        });
  }

  public Command DriveToFuel(Drive drive, Vision vision) {
    PathConstraints constraints =
        new PathConstraints(1, 1, Units.degreesToRadians(700), Units.degreesToRadians(1000));

    Pose2d desiredPose = vision.getFuelPose(new Pose3d(drive.getPose()));
    return AutoBuilder.pathfindToPose(desiredPose, constraints);
  }
}
