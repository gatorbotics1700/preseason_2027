package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

//what this command should do: turn the turret a certain number of degrees FROM ITS STARTING POINT using a PID and a two degree deadband
public class TurretCommand extends Command {
  private TurretSubsystem turretSubsystem;
  //TODO add any other variables here
  private PIDController pidController;
  private static final double kP = 0.2;
  private static final double kI = 0;
  private static final double kD = 0;

  public TurretCommand(TurretSubsystem turretSubsystem /*add any other inputs here (they should have matching variables above) */) {
    this.turretSubsystem = turretSubsystem;
    //TODO initialize any other inputs
    addRequirements(turretSubsystem);
    pidController = new PIDController(kP, kI, kD);
    //for reference, you can calculate a speed using the PID controller with the method pidController.calculate(currentPosition - desiredPosition);
  }

  @Override
  public void initialize() {
    //TODO anything you will need to do once at the start of the command
  }

  @Override
  public void execute() {
    //TODO any code you will want to run on a loop for the duration of the command
  }

  @Override
  public boolean isFinished() {
    if (/*TODO conditions for the command to be done */) {
      //TODO set the motor to 0 to be safe
      return true;
    }
    return false;
  }
}
