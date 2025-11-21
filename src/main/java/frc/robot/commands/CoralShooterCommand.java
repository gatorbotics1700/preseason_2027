// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralShooterCommand extends Command {
  CoralShooterSubsystem coralShooterSubsystem;
  double voltage;
  long startTime;

  public CoralShooterCommand(CoralShooterSubsystem coralShooterSubsystem, double voltage) {
    this.coralShooterSubsystem = coralShooterSubsystem;
    this.voltage = voltage;
    addRequirements(coralShooterSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
      
      coralShooterSubsystem.setMotorVoltage(voltage);
    
      if(voltage > 0){
        System.out.println("INTAKING!!!!!!!!!!");
      }
      else if(voltage == Constants.lFourShootingVoltage){
        System.out.println("SHOOTING L4!!!!!!");
      }
      else if(voltage == Constants.troughShootingVoltage){
        System.out.println("SHOOTING TROUGHHHHHH!!!!!");
      }
  }

  @Override
  public boolean isFinished() {
    // if we're intaking, we want to end either if it's been 5 seconds or the limit switch is triggered
      // the limit switch is triggered if the value it's getting is true
    // if our voltage is zero, we can just end, but to be safe also set speed to zero
    // if we're outtaking, we want to end if it's been 1.5 seconds
    // in each case, we want to print what's causing the command to end, 
      // set the motor voltage to 0, and return true
    // otherwise, we want to return false


    long timePassed = System.currentTimeMillis() - startTime;
      // you have the start time and can get the current time
      // this value is stored in milliseconds
    if(voltage > 0){
      if(timePassed == 5000 || coralShooterSubsystem.getLimitSwitchValue()){
          System.out.println("INTAKING FINISHED");
          coralShooterSubsystem.setMotorVoltage(0);
          return true;
      }
    }
    if(voltage == 0){
      System.out.println("ENDING...");
      coralShooterSubsystem.setMotorVoltage(0);
      return true;
    }

    if(voltage < 0){
      if(timePassed == 1500){
        System.out.println("OUTTAKING FINISHED");
        coralShooterSubsystem.setMotorVoltage(0);
        return true;
      }
    }
    
    return false;
  }
}
