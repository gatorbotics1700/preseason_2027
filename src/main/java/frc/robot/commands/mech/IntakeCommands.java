package frc.robot.commands.mech;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class IntakeCommands {

  public IntakeCommands() {}

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(true, intakeSubsystem);
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(false, intakeSubsystem);
  }

  public static class HomeIntakeDeploy extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public HomeIntakeDeploy(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDeployVoltage(IntakeConstants.HOMING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
      return intakeSubsystem.hallEffectTriggered();
    }

    @Override
    public void end(boolean interrupted) {
      intakeSubsystem.zeroIntakeDeploy();
      intakeSubsystem.setDesiredAngle(
          IntakeConstants.RETRACTED_POSITION.plus(new Rotation2d(Math.toRadians((2)))));
    }
  }

  public static Command RunIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setIntakeVoltage(IntakeConstants.INTAKING_VOLTAGE);
        });
  }

  public static Command StopIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
        () -> {
          intakeSubsystem.setIntakeVoltage(0);
        });
  }

  public static class DeployIntakeCommand extends Command {

    private final boolean isRetracting;
    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(boolean isRetracting, IntakeSubsystem intakeSubsystem) {
      this.isRetracting = isRetracting;
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
      if (isRetracting) {
        intakeSubsystem.retractDeployMotor();
      } else {
        intakeSubsystem.extendDeployMotor();
      }
    }

    @Override
    public boolean isFinished() {
      if(isRetracting && Math.abs(IntakeConstants.RETRACTED_ANGLE_DEGREES-intakeSubsystem.getCurrentAngle().getDegrees())<=IntakeConstants.POSITION_DEADBAND){
        return true;
      }
      if(!isRetracting && Math.abs(IntakeConstants.EXTENDED_ANGLE_DEGREES-intakeSubsystem.getCurrentAngle().getDegrees())<=IntakeConstants.POSITION_DEADBAND){
        return true;
      }
      return false;
    }
  }

  // TODO: does this go away now?
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

    return AutoBuilder.pathfindToPose(desiredPose, constraints);
    // return Commands.none();
  }
}
