package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ShooterSubsystem;

public class ShooterCommand extends Command{
    private ShooterSubsystem shooterSubsystem;
    private double flywheelVoltage;
    private boolean isTargetting;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double flywheelVoltage, boolean isTargetting){
        this.shooterSubsystem = shooterSubsystem;
        this.flywheelVoltage = flywheelVoltage;
        this.isTargetting = isTargetting;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        if(isTargetting){
            shooterSubsystem.setIsTargetting(true); //subsystem periodic will take care of moving hood
        } else{
            shooterSubsystem.setIsTargetting(false);
        }
    }

    @Override
    public void execute(){
        shooterSubsystem.setFlywheelVoltage(flywheelVoltage);
        // if(isTargetting){
        //     if(Math.abs(anglePosition - shooterSubsystem.getPosition()) > 2){
        //         hoodVoltage = pidController.calculate(anglePosition - shooterSubsystem.getPosition());
        //         shooterSubsystem.setHoodSpeed(hoodVoltage);
        //     } else{
        //         shooterSubsystem.setHoodSpeed(0);
        //     }
        // } else{
        if(isTargetting==false){ //if we aren't targetting then we want to retract the hood
            if(Math.abs(0 - shooterSubsystem.getPosition()) > 2){
                shooterSubsystem.turnToPosition(0);
            } else{
                shooterSubsystem.setHoodSpeed(0);
            }
        }
        // }
        
    }

    @Override
    public boolean isFinished(){
        if(flywheelVoltage==0){
            return true;
        }
        return false;
    }
}
