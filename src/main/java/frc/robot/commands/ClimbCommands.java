package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ClimberSubsystem;

public class ClimbCommands {

  private static final double L1_EXTENSION_INCHES = 20; // TODO get a real number

  private ClimbCommands() {}

  public Command ExtendClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, L1_EXTENSION_INCHES);
  }

  public Command RetractClimber(ClimberSubsystem climberSubsystem) {
    return new ClimberCommand(climberSubsystem, 0.0);
  }

  public Command Climb(ClimberSubsystem climberSubsystem) {
    return ExtendClimber(climberSubsystem)
        .alongWith(
            null) // TODO: add the align to tower (may add another command in this file to handle
        // driving to the tower)
        .andThen(RetractClimber(climberSubsystem));
  }

  private static class ClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final double height;

    ClimberCommand(ClimberSubsystem climberSubsystem, double height) {
      this.climberSubsystem = climberSubsystem;
      this.height = height;
      addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
      climberSubsystem.setDesiredPositionInches(height);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
      if (climberSubsystem.currentPositionInches() == 0) { // TODO probably add a deadband here?
        return true;
      }
      return false;
    }
  }
}
