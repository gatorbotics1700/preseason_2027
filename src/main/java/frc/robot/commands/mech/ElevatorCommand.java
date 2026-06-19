package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ElevatorSubsystem;

public class ElevatorCommand extends Command {
  private double rotations;
  private ElevatorSubsystem elevator;

  public ElevatorCommand(double rotations, ElevatorSubsystem elevator) {
    this.rotations = rotations;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.setTargetPose(rotations);
  }

  @Override
  public boolean isFinished() {
    return elevator.atTargetPose(rotations, 1.5); // got too lazy to make a constant...
  }
}
