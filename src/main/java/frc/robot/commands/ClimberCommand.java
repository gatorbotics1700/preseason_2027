package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ClimberSubsystem;

public class ClimberCommand extends Command {
  private final ClimberSubsystem climberSubsystem;
  private boolean extendingL1;
  private double desiredPosition;
  private final double L1_EXTENSION_INCHES = 20; // TODO get a real number

  public ClimberCommand(ClimberSubsystem climberSubsystem, boolean extendingL1) {
    this.climberSubsystem = climberSubsystem;
    this.extendingL1 = extendingL1;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.setDesiredPositionInches(L1_EXTENSION_INCHES);
    // drive to align with tower using path planner command stuff
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
