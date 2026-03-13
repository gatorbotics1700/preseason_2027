package frc.robot.commands.mech;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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
import frc.robot.util.Calculations;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotParameters;
import frc.robot.util.ValidStationaryShot;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootingCommands {
  public ShootingCommands() {}

  public static class ShootOnTheMoveCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Supplier<Pose2d> drivetrainPose;
    private Supplier<ChassisSpeeds> drivetrainVelocity;
    private final HopperFloorSubsystem hopperFloorSubsystem;
    private final HoodSubsystem hoodSubsystem;

    private final TurretSubsystem turretSubsystem;

    // Current logic is that if the flywheel speed is 0 then we're just tracking and if the flywheel
    // speed is not zero then we're trying to shoot, but we may decide we want a separate command
    // for
    // just tracking

    public ShootOnTheMoveCommand(
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
      setName("ShootOnTheMoveCommand");
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
      double desiredRotorVelocity = ShooterSubsystem.launchSpeedToRotorSpeed(params.shotSpeed);

      Logger.recordOutput(
          "Mech/ShotCalculator/shouldShoot", shooterSubsystem.getShouldShoot().getAsBoolean());
      Logger.recordOutput("Mech/ShootingCommand/validShot", params.shotSpeed != 0);
      Logger.recordOutput("Mech/ShootingCommand/shotSpeed", params.shotSpeed);
      Logger.recordOutput("Mech/ShootingCommand/rotorSpeed", desiredRotorVelocity);

      Logger.recordOutput("Mech/ShootingCommand/hoodAngle", params.hoodAngle.getDegrees());
      Logger.recordOutput("Mech/ShootingCommand/turretAngle", params.turretAngle.getDegrees());
      Logger.recordOutput("Mech/ShootingCommand/currentPose", drivetrainPose.get());
      Logger.recordOutput("Mech/ShootingCommand/chassisSpeeds", drivetrainVelocity.get());
      Logger.recordOutput("Mech/ShootingCommand/target", target);

      // if should be shooting
      // set flywheel speed to the last non-zero flywheel speed
      // if params.shotSpeed == 0 , stop the transition
      // if params.shotSpeed != 0 and our current speed matches that desired speed, run the
      // transition

      if (shooterSubsystem.getShouldShoot().getAsBoolean()) {
        if (params.shotSpeed != 0) { // and if we have a valid shot
          shooterSubsystem.setDesiredRotorVelocity(
              desiredRotorVelocity); // set velocity to our desired velocity
          hopperFloorSubsystem.setDesiredHopperFloorVoltage(
              HopperFloorConstants.HOPPER_FLOOR_VOLTAGE);
          if (Math.abs(shooterSubsystem.getFlywheelRotorVelocity() - desiredRotorVelocity)
              < ShooterConstants
                  .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired
            // velocity
            shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
          }
        } else { // if we dont have a valid shot
          hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
          shooterSubsystem.setDesiredTransitionVoltage(0);
        }
        hoodSubsystem.setDesiredAngle(
            hoodSubsystem.convertLaunchAngleToHoodAngle(params.hoodAngle));
        turretSubsystem.setDesiredAngle(params.turretAngle);
      } else {
        shooterSubsystem.setDesiredRotorVelocity(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
        hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      }
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    // the command only ends if another command requires the subsystems and it's interrupted, so set
    // these things in end instead of isFinished
    @Override
    public void end(boolean interrupted) {
      hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      shooterSubsystem.setDesiredTransitionVoltage(0);
    }
  }

  public static class ShootingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Supplier<Pose2d> drivetrainPose;
    private final HopperFloorSubsystem hopperFloorSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final ValidStationaryShot validStationaryShot;

    public ShootingCommand(
        ShooterSubsystem shooterSubsystem,
        HoodSubsystem hoodSubsystem,
        HopperFloorSubsystem hopperFloorSubsystem,
        Supplier<Pose2d> drivetrainPose,
        ValidStationaryShot validStationaryShot) {
      this.shooterSubsystem = shooterSubsystem;
      this.hopperFloorSubsystem = hopperFloorSubsystem;
      this.drivetrainPose = drivetrainPose;
      this.hoodSubsystem = hoodSubsystem;
      this.validStationaryShot = validStationaryShot;
      addRequirements(shooterSubsystem, hoodSubsystem, hopperFloorSubsystem);
      setName("ShootingCommand");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      // double desiredRotorVelocity =
      //     ShooterSubsystem.launchSpeedToRotorSpeed(validStationaryShot.shotSpeed);
      shooterSubsystem.setDesiredRotorVelocity(
          validStationaryShot.shotSpeed); // set velocity to our desired velocity
      hopperFloorSubsystem.setDesiredHopperFloorVoltage(HopperFloorConstants.HOPPER_FLOOR_VOLTAGE);
      if (Math.abs(shooterSubsystem.getFlywheelRotorVelocity() - validStationaryShot.shotSpeed)
          < ShooterConstants
              .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired
        // velocity
        shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
        hoodSubsystem.setDesiredAngle(
            hoodSubsystem.convertLaunchAngleToHoodAngle(validStationaryShot.hoodAngle));
      }
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      shooterSubsystem.setDesiredTransitionVoltage(0);
    }
  }

  public static Command StationaryShootingCommand(
      ShooterSubsystem shooterSubsystem,
      HoodSubsystem hoodSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> drivetrainPose) {
    ValidStationaryShot closestShot = null;
    Logger.recordOutput("Mech/Shooter/Stationary/RED_RIGHT", ShooterConstants.RED_RIGHT.pose);
    Logger.recordOutput("Mech/Shooter/Stationary/BLUE_LEFT", ShooterConstants.BLUE_LEFT.pose);
    Logger.recordOutput(
        "Mech/Shooter/Stationary/RED_RIGHT distance",
        Calculations.distanceToPoseInMeters(drivetrainPose.get(), ShooterConstants.RED_RIGHT.pose));
    Logger.recordOutput(
        "Mech/Shooter/Stationary/BLUE_LEFT distance",
        Calculations.distanceToPoseInMeters(drivetrainPose.get(), ShooterConstants.BLUE_LEFT.pose));
    for (ValidStationaryShot shot : ShooterConstants.STATIONARY_SHOT_ARRAY) {
      if (closestShot == null
          || Calculations.distanceToPoseInMeters(drivetrainPose.get(), shot.pose)
              < Calculations.distanceToPoseInMeters(drivetrainPose.get(), closestShot.pose)) {
        closestShot = shot;
      }
    }
    return AutoBuilder.pathfindToPose(
            closestShot.pose, new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
        .andThen(
            new ShootingCommand(
                shooterSubsystem, hoodSubsystem, hopperFloorSubsystem, drivetrainPose, closestShot))
        .withName("StationaryShootingCommand");
  }
}
