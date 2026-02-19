package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.TurretSubsystem;

/**
 * Runs the hood toward retract until the retracted limit switch is pressed, then zeros position.
 */
public class TurretHomingCommand extends Command {
  private final TurretSubsystem turretSubsystem;
  private double homingVoltage = 0.5; // TODO: change, plus check direction

  public TurretHomingCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TODO: CHECK IF SENSOR IS ON OR OFF WHEN SENSED
    if (!turretSubsystem.getHallEffectValue()) {
      turretSubsystem.setMotorVoltage(homingVoltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.homeTurret();
  }

  @Override
  public boolean isFinished() {
    return turretSubsystem.getHallEffectValue();
  }
}
