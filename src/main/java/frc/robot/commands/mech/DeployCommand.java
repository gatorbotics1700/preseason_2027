package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class DeployCommand extends Command {

  private final boolean isRetracting;
  private final IntakeSubsystem intakeSubsystem;

  public DeployCommand(boolean isRetracting, IntakeSubsystem intakeSubsystem) {
    this.isRetracting = isRetracting;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
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
    return true; // TODO fix
  }
}
