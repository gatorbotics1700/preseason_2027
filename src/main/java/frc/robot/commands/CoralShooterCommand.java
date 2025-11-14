// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralShooterCommand extends Command {
  private CoralShooterSubsystem coralShooterSubsystem;
  private final double voltage;
  private double startTime;

  /** Creates a new CoralShooterCommand. */
  public CoralShooterCommand(CoralShooterSubsystem coralShooterSubsystem, double voltage) {
    this.coralShooterSubsystem = coralShooterSubsystem;
    this.voltage = voltage;
    addRequirements(coralShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralShooterSubsystem.setMotorVoltage(voltage);

    if (voltage < 0) {
      System.out.println("INTAKING");
    } else if (voltage == Constants.L4_SHOOTING_VOLTAGE) {
      System.out.println("SHOOTING L4");
    } else if (voltage == Constants.TROUGH_SHOOTING_VOLTAGE) {
      System.out.println("SHOOTING TROUGH");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double timePassed = System.currentTimeMillis() - startTime;

    if (voltage > 0) {
      if (coralShooterSubsystem.getLimitSwitch()) {
        System.out.println("Limit switch triggered -- ending intake");
        coralShooterSubsystem.setMotorVoltage(0);
        return true;
      } else if (timePassed > 5000) {
        System.out.println("Finished intaking");
        coralShooterSubsystem.setMotorVoltage(0);
        return true;
      }
    } else if (voltage == 0) {
      System.out.println("MOTOR VOLTAGE IS 0, STOPPING");
      coralShooterSubsystem.setMotorVoltage(0);
      return true;
    } else if (voltage < 0) {
      if (timePassed > 1500) {
        System.out.println("Finished shooting");
        coralShooterSubsystem.setMotorVoltage(0);
        return true;
      }
    }
    return false;
  }
}
