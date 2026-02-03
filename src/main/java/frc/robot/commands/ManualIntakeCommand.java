package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class ManualIntakeCommand extends Command {
  private final double INTAKING_SPEED = 9; // TODO get a real number (I just picked my favorite)
  private final IntakeSubsystem intakeSubsystem;

  public ManualIntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.setIntakeSpeed(INTAKING_SPEED);
  }

  @Override
  public void execute() {
    // nothing, because gemma is going to do a super awesome job driving us to the balls
  }

  // TODO figure out how we want to end this command
  @Override
  public boolean isFinished() {
    return false;
  }
}
