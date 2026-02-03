package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HoodSubsystem;

public class HoodRetractCommand extends Command {
  private final HoodSubsystem hoodSubsystem;

  public HoodRetractCommand(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
    hoodSubsystem.setDesiredAngle(HoodSubsystem.RETRACTED_POSITION);
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
