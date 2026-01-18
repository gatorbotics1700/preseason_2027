package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ClimberSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.Constants;

public class ClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private int level;

    public ClimberCommand(ClimberSubsystem climberSubsystem, IntakeSubsystem intakeSubsystem, int level){
        this.climberSubsystem = climberSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.level = level;
        
        addRequirements(climberSubsystem, intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.extendIntake(false);
        if (level == 0){
            double groundPosition = climberSubsystem.getCurrentTicksOuter() - climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_LENGTH);
            climberSubsystem.extendOuterArm(groundPosition);
            System.out.println("RETURNING TO GROUND");
        } else if(level == 1){
            double LowPosition = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_LENGTH);
            climberSubsystem.extendOuterArm(LowPosition);
            double lowRetracted = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_RETRACTED);
            climberSubsystem.extendOuterArm(lowRetracted);
            System.out.println("AT L1");
        } else if (level == 2){

        } else if (level == 3){
            double lowPosition = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_LENGTH);
            climberSubsystem.extendOuterArm(lowPosition);

            double lowRetracted = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_RETRACTED);
            climberSubsystem.extendOuterArm(lowRetracted);

            double midPosition = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.MID_RUNG_ARM_LENGTH);
            climberSubsystem.extendInnerArm(midPosition);

            midPosition = climberSubsystem.getCurrentTicksInner() + climberSubsystem.InchesToTicks(Constants.MID_RUNG_ARM_LENGTH);  //check if arms will have to extend and retract different lengths
            climberSubsystem.extendOuterArm(midPosition);

            double midRetracted = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.MID_RUNG_ARM_RETRACTED);
            climberSubsystem.extendOuterArm(midRetracted);

            midRetracted = climberSubsystem.getCurrentTicksInner() + climberSubsystem.InchesToTicks(Constants.MID_RUNG_ARM_RETRACTED);
            climberSubsystem.extendInnerArm(midRetracted);

            double highPosition = climberSubsystem.getCurrentTicksInner() + climberSubsystem.InchesToTicks(Constants.HIGH_RUNG_ARM_LENGTH);
            climberSubsystem.extendInnerArm(highPosition);

            highPosition = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.HIGH_RUNG_ARM_LENGTH);
            climberSubsystem.extendOuterArm(highPosition);

            double highRetracted = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.HIGH_RUNG_ARM_RETRACTED);
            climberSubsystem.extendOuterArm(highRetracted);
            System.out.println("AT L3!!!");
        } else {
            System.out.println("*** NOT VALID LEVEL ***");
        }
    }

}
