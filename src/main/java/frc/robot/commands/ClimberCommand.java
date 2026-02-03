package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ClimberSubsystem;

public class ClimberCommand extends Command {
  private final ClimberSubsystem climberSubsystem;
  private final double L1_EXTENSION_INCHES = 20; // TODO get a real number

  public ClimberCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.setDesiredPositionInches(L1_EXTENSION_INCHES);
    // drive to align with tower using path planner command stuff
    // TODO decide if we want to split out the driving into a separate command and make the set
    // extension ones instant commands
    climberSubsystem.setDesiredPositionInches(0); // retracts the climber to its zero position
  }

  @Override
  public boolean isFinished() {
    if (climberSubsystem.currentPositionInches() == 0) { // TODO probably add a deadband here?
      return true;
    }
    return false;
  }
}
