package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;

  private final double intakeVoltage;
  private final boolean wantExtended;

  public IntakeCommand(
      IntakeSubsystem intakeSubsystem, double intakeVoltage, boolean wantExtended) {
    this.intakeSubsystem = intakeSubsystem;
    this.intakeVoltage = intakeVoltage;
    this.wantExtended = wantExtended;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.pivotIntake(wantExtended);
    intakeSubsystem.setIntakeVoltage(intakeVoltage);
  }

  @Override
  public boolean isFinished() {
    if (intakeVoltage == 0) {
      return true;
    }
    // potentially consider adding connection to vision, if we don't see any fuel
    return false;
  }
}
