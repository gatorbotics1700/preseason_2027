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

  private IntakeCommands() {}

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(true, intakeSubsystem);
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(false, intakeSubsystem);
  }

  private static class HomeIntakeDeploy extends Command {
    private final IntakeSubsystem intakeSubsystem;

    HomeIntakeDeploy(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDeployVoltage(IntakeConstants.HOMING_VOLTAGE);
    }

    @Override
    public void execute() {}

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
      return true; // TODO fix
    }
  }
}
