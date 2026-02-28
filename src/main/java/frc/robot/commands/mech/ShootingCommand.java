package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotParameters;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootingCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private Supplier<Pose2d> drivetrainPose;
  private Supplier<ChassisSpeeds> drivetrainVelocity;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final HoodSubsystem hoodSubsystem;

  private final TurretSubsystem turretSubsystem;

  // Current logic is that if the flywheel speed is 0 then we're just tracking and if the flywheel
  // speed is not zero then we're trying to shoot, but we may decide we want a separate command for
  // just tracking

  public ShootingCommand(
      ShooterSubsystem shooterSubsystem,
      HoodSubsystem hoodSubsystem,
      TurretSubsystem turretSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> drivetrainPose,
      Supplier<ChassisSpeeds> drivetrainVelocity) {

    this.shooterSubsystem = shooterSubsystem;
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.drivetrainPose = drivetrainPose;
    this.drivetrainVelocity = drivetrainVelocity;
    this.hoodSubsystem = hoodSubsystem;
    this.turretSubsystem = turretSubsystem;
    addRequirements(shooterSubsystem, hoodSubsystem, turretSubsystem, hopperFloorSubsystem);
    setName("ShootingCommand");
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation3d target;
    if (FieldCoordinates.BLUE_BUMP_AND_TRENCH_X <= drivetrainPose.get().getX()
        && drivetrainPose.get().getX() < FieldCoordinates.RED_BUMP_AND_TRENCH_X) {
      if (FieldCoordinates.FIELD_CENTER.getY() < drivetrainPose.get().getY()) {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldCoordinates.BLUE_RIGHT_FUNNELING
                : FieldCoordinates.RED_LEFT_FUNNELING;

      } else {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? FieldCoordinates.BLUE_LEFT_FUNNELING
                : FieldCoordinates.RED_RIGHT_FUNNELING;
      }

    } else {
      target =
          DriverStation.getAlliance().get() == Alliance.Blue
              ? FieldCoordinates.BLUE_HUB
              : FieldCoordinates.RED_HUB;
    }
    ShotParameters params =
        ShotCalculator.calculateShot(drivetrainPose.get(), drivetrainVelocity.get(), target);
    double desiredFlywheelSpeed = ShooterSubsystem.calculateFlywheelSpeed(params.shotSpeed);

    Logger.recordOutput(
        "Mech/ShotCalculator/shouldShoot", shooterSubsystem.getShouldShoot().getAsBoolean());
    Logger.recordOutput("Mech/ShotCalculator/validShot", params.shotSpeed != 0);
    Logger.recordOutput("Mech/ShotCalculator/shotSpeed", params.shotSpeed);
    Logger.recordOutput("Mech/ShotCalculator/flyWheelSpeed", desiredFlywheelSpeed);

    Logger.recordOutput("Mech/ShotCalculator/hoodAngle", params.hoodAngle.getDegrees());
    Logger.recordOutput("Mech/ShotCalculator/turretAngle", params.turretAngle.getDegrees());
    Logger.recordOutput("Mech/ShotCalculator/currentPose", drivetrainPose.get());
    Logger.recordOutput("Mech/ShotCalculator/chassisSpeeds", drivetrainVelocity.get());
    Logger.recordOutput("Mech/ShotCalculator/target", target);

    // if should be shooting
    // set flywheel speed to the last non-zero flywheel speed
    // if params.shotSpeed == 0 , stop the transition
    // if params.shotSpeed != 0 and our current speed matches that desired speed, run the transition

    if (shooterSubsystem.getShouldShoot().getAsBoolean()) {
      System.out.println("WE WANT TO SHOOT");
      if (params.shotSpeed != 0) { // and if we have a valid shot

        System.out.println("VALID SHOT VALID SHOT");
        shooterSubsystem.setDesiredFlywheelVelocity(
            desiredFlywheelSpeed); // set velocity to our desired velocity
        hopperFloorSubsystem.setDesiredHopperFloorVelocity(
            HopperFloorConstants.HOPPER_FLOOR_VELOCITY);
        if (Math.abs(shooterSubsystem.getFlywheelVelocity() - desiredFlywheelSpeed)
            < ShooterConstants
                .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired
          // velocity
          // System.out.println("SHOOTING SHOOTING SHOOTING");
          shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
        }
      } else { // if we dont have a valid shot
        // System.out.println("INVALID SHOT INVALID SHOT");
        hopperFloorSubsystem.setDesiredHopperFloorVelocity(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
      }
      hoodSubsystem.setDesiredAngle(params.hoodAngle);
      turretSubsystem.setDesiredAngle(params.turretAngle);
    } else {
      System.out.println("WE DONT WANT TO SHOOT");
      shooterSubsystem.setDesiredFlywheelVelocity(0);
      shooterSubsystem.setDesiredTransitionVoltage(0);
      hopperFloorSubsystem.setDesiredHopperFloorVelocity(0);
    }

    // TODO add drivetrain angle things here instead of the turret angle for testing on sting
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // the command only ends if another command requires the subsystems and it's interrupted, so set
  // these things in end instead of isFinished
  @Override
  public void end(boolean interrupted) {
    hopperFloorSubsystem.setDesiredHopperFloorVelocity(0);
    shooterSubsystem.setDesiredTransitionVoltage(0);
  }
}
