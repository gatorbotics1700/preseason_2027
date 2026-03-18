package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;

public class DriveSystemsCheckCommands {
  private DriveSystemsCheckCommands() {}

  public static class DriveForThreeSeconds extends Command {
    // include any subsystem requirements here
    private final Drive drive;
    private double startTimeMillis;
    private ChassisSpeeds chassisSpeeds;

    // constructor
    public DriveForThreeSeconds(Drive drive, ChassisSpeeds chassisSpeeds) {
      this.drive = drive;
      this.chassisSpeeds = chassisSpeeds;
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      startTimeMillis = System.currentTimeMillis();
    }

    @Override
    public void execute() {
      drive.runVelocity(chassisSpeeds);
    }

    @Override
    public boolean isFinished() {
      if (System.currentTimeMillis() - startTimeMillis > 3000) {
        drive.stop();
        return true;
      }
      return false;
    }
  }

  public static Command DriveSystemCheckCommand(Drive drive) {
    return new DriveForThreeSeconds(
            drive, ChassisSpeeds.fromFieldRelativeSpeeds(0.8, 0, 0, drive.getRotation()))
        .andThen(new WaitCommand(1))
        .andThen(
            new DriveForThreeSeconds(
                drive, ChassisSpeeds.fromFieldRelativeSpeeds(0, 0.8, 0, drive.getRotation())))
        .andThen(new WaitCommand(1))
        .andThen(
            new DriveForThreeSeconds(
                drive, ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 1, drive.getRotation())));
  }
}
