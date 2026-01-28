package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.mech.ClimberSubsystem;

public class ClimberCommand extends Command {
  private final ClimberSubsystem climberSubsystem;
  private boolean extendingL1;
  private double desiredPosition;

  public ClimberCommand(ClimberSubsystem climberSubsystem, boolean extendingL1) {
    this.climberSubsystem = climberSubsystem;
    this.extendingL1 = extendingL1;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    if (extendingL1 == true) {
      desiredPosition = climberSubsystem.inchesToTicks(Constants.CLIMBER_EXTENDED_POSITION);
      System.out.println("EXTENDING CLIMBER");
    } else {
      desiredPosition = climberSubsystem.inchesToTicks(Constants.CLIMBER_RETRACTED_POSITION);
      System.out.println("RETRACTING CLIMBER");
    }
  }

  @Override
  public void execute() {
    climberSubsystem.moveArm(desiredPosition);
  }

  @Override
  public boolean isFinished() {
    if (climberSubsystem.getMotorOutput() == 0) {
      return true;
    }
    return false;
  }
}
