package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeDeployCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;

  public IntakeDeployCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setDesiredangle(intakeSubsystem.EXTENDED_POSITION);
  }

  @Override
  public void execute() {}

  // TODO figure out if this should be an instant command because I don't know enough about command
  // based
  @Override
  public boolean isFinished() {
    return true;
  }
}
