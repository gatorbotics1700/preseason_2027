package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.mech.ElevatorSubsystem;

public class ElevatorCommands extends Command{
    
    private ElevatorSubsystem elevatorSubsystem;
    private double rotations;

    public ElevatorCommands(double rotations, ElevatorSubsystem elevatorSubsystem) {
        this.rotations = rotations;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }


    @Override
    public void execute() {
        elevatorSubsystem.setDesiredPosition(rotations);
    }

    @Override
    public boolean isFinished(){
        if(elevatorSubsystem.atDesiredPosition(rotations, ElevatorConstants.ERROR) == true){
            return true;
        }
        return false;
    }
}