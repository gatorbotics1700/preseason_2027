package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommands {

  public IntakeCommands() {}

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) { // TODO use hall effect
    return new DeployIntakeCommand(true, intakeSubsystem).withName("Retract Intake");
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(false, intakeSubsystem).withName("Deploy Intake");
  }

  public static class HomeIntakeDeploy extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public HomeIntakeDeploy(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Home Intake Deploy");
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDeployVoltage(IntakeConstants.HOMING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
      return intakeSubsystem.isHallEffectTriggered();
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
            })
        .withName("Run Intake");
  }

  public static Command StopIntake(IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
            () -> {
              intakeSubsystem.setIntakeVoltage(0);
            })
        .withName("Stop Intake");
  }

  public static class DeployIntakeCommand extends Command {

    private final boolean isRetracting;
    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(boolean isRetracting, IntakeSubsystem intakeSubsystem) {
      this.isRetracting = isRetracting;
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Deploy Intake");
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
      if (isRetracting
          && Math.abs(
                  IntakeConstants.RETRACTED_ANGLE_DEGREES
                      - intakeSubsystem.getCurrentAngle().getDegrees())
              <= IntakeConstants.POSITION_DEADBAND) {
        return true;
      }
      if (!isRetracting
          && Math.abs(
                  IntakeConstants.EXTENDED_ANGLE_DEGREES
                      - intakeSubsystem.getCurrentAngle().getDegrees())
              <= IntakeConstants.POSITION_DEADBAND) {
        return true;
      }
      return false;
    }
  }
}
