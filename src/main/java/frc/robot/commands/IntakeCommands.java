package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommands {

    private final double INTAKING_SPEED = 9; // TODO get a real number (I just picked my favorite)

    private IntakeCommands() {}

    public Command RetractIntake(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(
            () -> {
                intakeSubsystem.setDesiredangle(intakeSubsystem.RETRACTED_POSITION);
            }
        );
    }

    public Command DeployIntake(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(
            () -> {
                intakeSubsystem.setDesiredangle(intakeSubsystem.EXTENDED_POSITION);
            }
        );
    }

    public Command RunIntake(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(
            () -> {
                intakeSubsystem.setIntakeSpeed(INTAKING_SPEED);
            }
        );
    }

    public Command StopIntake(IntakeSubsystem intakeSubsystem) {
        return new InstantCommand(
            () -> {
                intakeSubsystem.setIntakeSpeed(0);
            }
        );
    }

    //TODO: add drive to fuel in this file
}
