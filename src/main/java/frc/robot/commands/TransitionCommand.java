package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.TransitionSubsystem;

public class TransitionCommand extends Command {

  private final TransitionSubsystem transitionSubsystem;

  private final double kickerVoltage;
  private final double hopperVoltage;

  public TransitionCommand(
      TransitionSubsystem transitionSubsystem, double kickerVoltage, double hopperVoltage) {
    this.transitionSubsystem = transitionSubsystem;
    this.kickerVoltage = kickerVoltage;
    this.hopperVoltage = hopperVoltage;
    addRequirements(transitionSubsystem);
  }

  @Override
  public void execute() {
    transitionSubsystem.setLowKickerVoltage(kickerVoltage);
    transitionSubsystem.setHighKickerVoltage(kickerVoltage);
    transitionSubsystem.setHopperVoltage(hopperVoltage);
  }

  @Override
  public boolean isFinished() {
    if (hopperVoltage == 0) {
      return true;
    }
    return false;
  }
}
