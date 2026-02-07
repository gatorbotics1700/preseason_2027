package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.util.ShotParameters;
import java.util.function.Supplier;

public class ShootingCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private Supplier<Pose2d> drivetrainPose;
  private Supplier<ChassisSpeeds> drivetrainVelocity;
  private Translation3d target;
  private double flywheelSpeed;
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
      double flywheelSpeed,
      Supplier<Pose2d> drivetrainPose,
      Supplier<ChassisSpeeds> drivetrainVelocity,
      Translation3d target) {

    this.shooterSubsystem = shooterSubsystem;
    this.hopperFloorSubsystem = hopperFloorSubsystem;
    this.flywheelSpeed = flywheelSpeed;
    this.drivetrainPose = drivetrainPose;
    this.drivetrainVelocity = drivetrainVelocity;
    this.target = target;
    this.hoodSubsystem = hoodSubsystem;
    this.turretSubsystem = turretSubsystem;
    addRequirements(shooterSubsystem, hoodSubsystem, turretSubsystem, hopperFloorSubsystem);
  }

  @Override
  public void initialize() {
    // only want to run hopper if we're actively trying to shoot
    if (flywheelSpeed != 0) {
      hopperFloorSubsystem.setHopperFloorVelocity(HopperFloorSubsystem.HOPPER_FLOOR_SPEED);
    }
  }

  @Override
  public void execute() {
    // calculate angles and get the hood and turret to track
    ShotParameters params = new ShotParameters(new Rotation2d(), new Rotation2d());
    // ShotCalculator.calculateShot(drivetrainPose.get(), drivetrainVelocity.get(), target);
    hoodSubsystem.setDesiredAngle(params.hoodAngle);
    turretSubsystem.setDesiredAngle(params.turretAngle);

    // set the flywheel desired speed
    shooterSubsystem.setFlywheelVelocity(flywheelSpeed);

    // if we're actually trying to shoot, and the flywheel is up to speed, kick the balls into the
    // shooter!
    // TODO: add a way to indicate whether we're actually shooting, if we are, run kicker, but
    // otherwise always be running flywheel
    // TODO: actually shooting should be based on robotPose in the alliance zone, so it doesnt have
    // to be a button
    // TODO: add funnel command (separate command) & instant command to stop running the flywheel
    if (flywheelSpeed != 0
        && shooterSubsystem.getFlywheelVelocity() == flywheelSpeed) { // TODO add a deadband probably
      shooterSubsystem.setTransitionSpeed(ShooterSubsystem.TRANSITION_SPEED);
    } else {
      shooterSubsystem.setTransitionSpeed(0);
    }
  }

  // TODO figure out if we want a way to end this command
  @Override
  public boolean isFinished() {
    return false;
  }
}
