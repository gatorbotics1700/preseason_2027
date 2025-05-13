package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestCommand extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final Pose2d desiredPose;
   

    public TestCommand(DrivetrainSubsystem drivetrain,
            Pose2d desiredPose) {
        this.drivetrain = drivetrain;
        this.desiredPose = desiredPose;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        drivetrain.resetPose(new Pose2d(0,0,new Rotation2d(0)));
    }

    @Override
    public void execute() {
        drivetrain.driveToPose(desiredPose);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}