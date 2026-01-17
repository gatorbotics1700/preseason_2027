package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final double voltage;
    private final boolean wantExtended;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double voltage, boolean wantExtended){
        this.intakeSubsystem = intakeSubsystem;
        this.voltage = voltage;
        this.wantExtended = wantExtended;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.extendIntake(wantExtended);
        intakeSubsystem.setMotorVoltage(voltage);
    }

    @Override
    public boolean isFinished(){
        if(voltage == 0){
            return true;
        }
        //potentially consider adding connection to vision, if we don't see any fuel
        return false;
    }

}
