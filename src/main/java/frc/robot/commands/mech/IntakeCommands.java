package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommands {

  public IntakeCommands() {}

  public static Command ToggleIntake(IntakeSubsystem intakeSubsystem) {
    if (intakeSubsystem.getIsDeployed().getAsBoolean()) {
      return RetractIntake(intakeSubsystem);
    } else {
      return DeployIntake(intakeSubsystem);
    }
  }

  public static Command RetractIntake(IntakeSubsystem intakeSubsystem) {
    return new RetractIntakeCommand(intakeSubsystem)
        .andThen(new WaitCommand(0.75))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIsDeployedToFalse()))
        .withName("Retract Intake");
  }

  public static Command DeployIntake(IntakeSubsystem intakeSubsystem) {
    return new DeployIntakeCommand(intakeSubsystem)
        .andThen(new WaitCommand(0.75))
        .andThen(new InstantCommand(() -> intakeSubsystem.setIsDeployedToTrue()))
        .withName("Deploy Intake");
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
      intakeSubsystem.setIsDeployedToFalse();
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

    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Deploy Intake");
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDesiredAngle(IntakeConstants.EXTENDED_POSITION);
    }

    @Override
    public boolean isFinished() {
      if (Math.abs(
              IntakeConstants.EXTENDED_ANGLE_DEGREES
                  - intakeSubsystem.getCurrentAngle().getDegrees())
          <= IntakeConstants.POSITION_DEADBAND) {
        return true;
      }
      return false;
    }
  }

  public static class RetractIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public RetractIntakeCommand(IntakeSubsystem intakeSubsystem) {
      this.intakeSubsystem = intakeSubsystem;
      addRequirements(intakeSubsystem);
      setName("Retract Intake");
    }

    @Override
    public void initialize() {
      intakeSubsystem.setDesiredAngle(IntakeConstants.RETRACTED_POSITION);
    }

    @Override
    public boolean isFinished() {
      return intakeSubsystem.isHallEffectTriggered()
          || (Math.abs(
                  IntakeConstants.RETRACTED_ANGLE_DEGREES
                      - intakeSubsystem.getCurrentAngle().getDegrees())
              <= IntakeConstants.POSITION_DEADBAND);
    }

    @Override
    public void end(boolean interrupted) {
      if (!intakeSubsystem.isHallEffectTriggered()) {
        new IntakeCommands.HomeIntakeDeploy(intakeSubsystem);
      } else {
        intakeSubsystem.zeroIntakeDeploy();
      }
    }
  }
}
