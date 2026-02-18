package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HoodSubsystem;

/**
 * Runs the hood toward retract until the retracted limit switch is pressed, then zeros position.
 */
public class HomeHoodCommand extends Command {
  private final HoodSubsystem hoodSubsystem;

  public HomeHoodCommand(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
    hoodSubsystem.setRetractingToLimitSwitch(true);
  }

  @Override
  public void execute() {
    hoodSubsystem.setHoodVoltage(HoodSubsystem.RETRACT_TO_LIMIT_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.setRetractingToLimitSwitch(false);
  }

  @Override
  public boolean isFinished() {
    return hoodSubsystem.isRetractedLimitSwitchPressed();
  }
}
