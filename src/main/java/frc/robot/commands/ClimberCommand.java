package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ClimberSubsystem;
import frc.robot.Constants;

public class ClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private boolean extendingL1;
    private double desiredPosition;
    private static final double DEADBAND = 200; //TODO: change


    public ClimberCommand(ClimberSubsystem climberSubsystem, boolean extendingL1){
        this.climberSubsystem = climberSubsystem;
        this.extendingL1 = extendingL1;
        
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize(){
        if(extendingL1 == true){
            desiredPosition = climberSubsystem.inchesToTicks(Constants.CLIMBER_EXTENDED_POSITION);
            System.out.println("EXTENDING CLIMBER");
        } else{
            desiredPosition = climberSubsystem.inchesToTicks(Constants.CLIMBER_RETRACTED_POSITION);
            System.out.println("RETRACTING CLIMBER");
        }
    }

    @Override
    public void execute(){
        climberSubsystem.moveArm(desiredPosition);
    }

    @Override
    public boolean isFinished(){
        double error = climberSubsystem.getCurrentTicks() - desiredPosition;
        if(error < DEADBAND){
            climberSubsystem.setMotorOutput(0);
            System.out.println("CLIMBER REACHED DESIRED POSITION");
            return true;
        } else if (climberSubsystem.getMotorOutput() == 0){
            climberSubsystem.setMotorOutput(0);
            System.out.println("CLIMBER NOT MOVING");
            return true;
        }
        return false;
    }
}