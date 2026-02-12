package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {

  private static final double INTAKING_SPEED =
      9; // TODO get a real number (I just picked my favorite)

  private IntakeCommands() {}

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setDesiredangle(intakeSubsystem.RETRACTED_POSITION);
        });
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setDesiredangle(intakeSubsystem.EXTENDED_POSITION);
        });
  }

  public static Command RunIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setIntakeSpeed(INTAKING_SPEED);
        });
  }

  public static Command StopIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setIntakeSpeed(0);
        });
  }

  public static Command DriveToFuel(Drive drive, Vision vision) {
    PathConstraints constraints =
        new PathConstraints(1, 2, Units.degreesToRadians(700), Units.degreesToRadians(1000));

    Pose2d currentPose = drive.getPose();
    Pose2d desiredPose = vision.getFuelPose(currentPose);
    Logger.recordOutput("Odometry/Desired Pose in Intake", desiredPose);
    Logger.recordOutput("Odometry/Current Pose in Intake", currentPose);
    // Pose2d desiredPose = new Pose2d(5, 7, new Rotation2d());
    System.out.println("this is the desired pose here u go: " + desiredPose);
    if (desiredPose == null) {
      return Commands.none();
    }

    // return AutoBuilder.pathfindToPose(desiredPose, constraints);
    return Commands.none();
  }
}
