package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ShooterSubsystem;

public class ShooterCommand extends Command {
  private ShooterSubsystem shooterSubsystem;
  private double flywheelVoltage;
  private double kickerVoltage;
  private double startTime;

  public ShooterCommand(ShooterSubsystem shooterSubsystem, double flywheelVoltage) {
    this.shooterSubsystem = shooterSubsystem;
    this.flywheelVoltage = flywheelVoltage;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    shooterSubsystem.setFlywheelVoltage(flywheelVoltage);
    if (Math.abs(flywheelVoltage) > 0) {
      System.out.println("SHOOTING SHOOTING SHOOTING");
    } else {
      System.out.println("STOPPING");
    }
  }

  @Override
  public boolean isFinished() {
    double timePassed = System.currentTimeMillis() - startTime;
    if (flywheelVoltage == 0) {
      shooterSubsystem.setFlywheelVoltage(0);
      return true;
    }
    if (timePassed > 8000) {
      shooterSubsystem.setFlywheelVoltage(0);
      System.out.println("TIMING OUT");
      return true;
    }
    return false;
  }
}
