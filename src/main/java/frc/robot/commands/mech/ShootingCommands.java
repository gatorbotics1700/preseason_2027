package frc.robot.commands.mech;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    // Current logic is that if the flywheel speed is 0 then we're just tracking and if the flywheel
    // speed is not zero then we're trying to shoot, but we may decide we want a separate command
    // for
    // just tracking

  public static class ShootingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Supplier<Pose2d> drivetrainPose;
    private final HopperFloorSubsystem hopperFloorSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final ShotParameters shotParameters;
    //private final TurretSubsystem turretSubsystem;

    public ShootingCommand(
        ShooterSubsystem shooterSubsystem,
        HoodSubsystem hoodSubsystem,
        HopperFloorSubsystem hopperFloorSubsystem,
        //TurretSubsystem turretSubsystem,
        Supplier<Pose2d> drivetrainPose,
        ShotParameters shotParameters) {
      this.shooterSubsystem = shooterSubsystem;
      this.hopperFloorSubsystem = hopperFloorSubsystem;
      this.drivetrainPose = drivetrainPose;
      this.hoodSubsystem = hoodSubsystem;
      this.shotParameters = shotParameters;
      //this.turretSubsystem = turretSubsystem;
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
          shotParameters.shotSpeed); // set velocity to our desired velocity
      hopperFloorSubsystem.setDesiredHopperFloorVoltage(HopperFloorConstants.HOPPER_FLOOR_VOLTAGE);
      if (Math.abs(shooterSubsystem.getFlywheelRotorVelocity() - shotParameters.shotSpeed)
          < ShooterConstants
              .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired velocity
        //turretSubsystem.setDesiredAngle(shotParameters.turretAngle);
        hoodSubsystem.setDesiredAngle(hoodSubsystem.convertLaunchAngleToHoodAngle(shotParameters.hoodAngle));
        shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
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

  public static class StopShooting extends Command{
    private final ShooterSubsystem shooterSubsystem;
    private final HopperFloorSubsystem hopperFloorSubsystem;

    public StopShooting(
        ShooterSubsystem shooterSubsystem,
        HopperFloorSubsystem hopperFloorSubsystem) {
      this.shooterSubsystem = shooterSubsystem;
      this.hopperFloorSubsystem = hopperFloorSubsystem;
      addRequirements(shooterSubsystem, hopperFloorSubsystem);
      setName("StopShooting");
    }
  
    @Override
    public void initialize(){}

    @Override
    public void execute(){
      if(shooterSubsystem.getFlywheelRotorVelocity()!=0){
        shooterSubsystem.setDesiredRotorVelocity(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
        hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
      } else{
      System.out.println("SHOOTER ALREADY NOT MOVING");
      }
    }

    @Override
    public boolean isFinished(){
      if (shooterSubsystem.getFlywheelRotorVelocity() == 0){
        return true;
      }
      return false;
    }
  }
  

  public static Command StationaryShootingCommand(
      ShooterSubsystem shooterSubsystem,
      HoodSubsystem hoodSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      Supplier<Pose2d> drivetrainPose) {
    ShotParameters closestShotParameters = null;
    Logger.recordOutput("Mech/Shooter/Stationary/RED_RIGHT", ShooterConstants.RED_RIGHT.pose);
    Logger.recordOutput("Mech/Shooter/Stationary/BLUE_LEFT", ShooterConstants.BLUE_LEFT.pose);
    Logger.recordOutput(
        "Mech/Shooter/Stationary/RED_RIGHT distance",
        Calculations.distanceToPoseInMeters(drivetrainPose.get(), ShooterConstants.RED_RIGHT.pose));
    Logger.recordOutput(
        "Mech/Shooter/Stationary/BLUE_LEFT distance",
        Calculations.distanceToPoseInMeters(drivetrainPose.get(), ShooterConstants.BLUE_LEFT.pose));
    for (ShotParameters shot : ShooterConstants.STATIONARY_SHOT_ARRAY) {
      if (closestShotParameters == null
          || Calculations.distanceToPoseInMeters(drivetrainPose.get(), shot.pose)
              < Calculations.distanceToPoseInMeters(drivetrainPose.get(), closestShotParameters.pose)) {
        closestShotParameters = shot;
      }
    }
    return AutoBuilder.pathfindToPose(
            closestShotParameters.pose, new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
        .andThen(
            new ShootingCommand(
                shooterSubsystem, hoodSubsystem, hopperFloorSubsystem, drivetrainPose, closestShotParameters))
        .withName("StationaryShootingCommand");
  }

  public static Command ShootOnTheMoveCommand(
        ShooterSubsystem shooterSubsystem,
        HoodSubsystem hoodSubsystem,
        HopperFloorSubsystem hopperFloorSubsystem,
        TurretSubsystem turretSubsystem,
        Supplier<Pose2d> drivetrainPose,
        Supplier<ChassisSpeeds> drivetrainVelocity) {
      
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
          ShotCalculator.calculateShot(drivetrainPose.get(), drivetrainVelocity.get(), target); //calculates the shot params with turretAngle
      double desiredRotorVelocity = ShooterSubsystem.launchSpeedToRotorSpeed(params.shotSpeed);

      Logger.recordOutput("Mech/ShootingCommand/validShot", params.shotSpeed != 0);
      Logger.recordOutput("Mech/ShootingCommand/shotSpeed", params.shotSpeed);
      Logger.recordOutput("Mech/ShootingCommand/rotorSpeed", desiredRotorVelocity);

      Logger.recordOutput("Mech/ShootingCommand/hoodAngle", params.hoodAngle.getDegrees());
      Logger.recordOutput("Mech/ShootingCommand/turretAngle", params.turretAngle.getDegrees());
      Logger.recordOutput("Mech/ShootingCommand/currentPose", drivetrainPose.get());
      Logger.recordOutput("Mech/ShootingCommand/chassisSpeeds", drivetrainVelocity.get());
      Logger.recordOutput("Mech/ShootingCommand/target", target);

      if (params.shotSpeed == 0) { // if we dont have a valid shot
        hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
        params.turretAngle = new Rotation2d(0);
        params.hoodAngle = new Rotation2d(0);
        params.shotSpeed = 0;
      }
      turretSubsystem.setDesiredAngle(params.turretAngle);
      return new ShootingCommand(shooterSubsystem, hoodSubsystem, hopperFloorSubsystem, drivetrainPose, params);

  }
} 
