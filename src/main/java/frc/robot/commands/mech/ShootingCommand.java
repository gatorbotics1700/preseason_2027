package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotParameters;
import java.util.function.Supplier;

public class ShootingCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private Supplier<Pose2d> drivetrainPose;
  private Supplier<ChassisSpeeds> drivetrainVelocity;
  private Translation3d target;
  private final HopperFloorSubsystem hopperFloorSubsystem;
  private final HoodSubsystem hoodSubsystem;

  private final Translation3d targetRight = new Translation3d(10.0, 50.0, 0.0);
  private final Translation3d targetLeft = new Translation3d(10.0, 10.0, 0.0);

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
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Translation3d target;
    if (Constants.BLUE_BUMP_AND_TRENCH_X <= drivetrainPose.get().getX()
        && drivetrainPose.get().getX() < Constants.RED_BUMP_AND_TRENCH_X) {
      if (Constants.FIELD_CENTER.getY() < drivetrainPose.get().getY()) {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? Constants.BLUE_RIGHT_FUNNELING
                : Constants.RED_LEFT_FUNNELING;

      } else {
        target =
            DriverStation.getAlliance().get() == Alliance.Blue
                ? Constants.BLUE_LEFT_FUNNELING
                : Constants.RED_RIGHT_FUNNELING;
      }

    } else {
      target =
          DriverStation.getAlliance().get() == Alliance.Blue
              ? Constants.BLUE_HUB
              : Constants.RED_HUB;
    }
    ShotParameters params =
        ShotCalculator.calculateShot(drivetrainPose.get(), drivetrainVelocity.get(), target);
    if (shooterSubsystem.getShouldShoot()) {
      // calculate angles and get the hood and turret to track

      // only want to run hopper if we're actively trying to shoot
      // TODO figure out if this is the logic we actually want
      if (params.shotSpeed != 0) {
        hopperFloorSubsystem.setHopperFloorVelocity(HopperFloorSubsystem.HOPPER_FLOOR_SPEED);
        shooterSubsystem.setFlywheelVelocity(
            ShooterSubsystem.calculateFlywheelSpeed(params.shotSpeed));
        if (Math.abs(
                shooterSubsystem.getFlywheelVelocity()
                    - ShooterSubsystem.calculateFlywheelSpeed(params.shotSpeed))
            < shooterSubsystem.FLYWHEEL_SPEED_DEADBAND) {
          shooterSubsystem.setDesiredTransitionVoltage(ShooterSubsystem.TRANSITION_VOLTAGE);
        }
      } else {
        shooterSubsystem.setDesiredTransitionVoltage(0);
        hopperFloorSubsystem.setHopperFloorVelocity(0);
      }
      // TODO add drivetrain angle things here instead of the turret angle for testing on sting

      // since angle calculations for the shot are ground-rleative

      // if we're actually trying to shoot, and the flywheel is up to speed, kick the balls into the
      // shooter!
      // TODO: add a way to indicate whether we're actually shooting, if we are, run kicker, but
      // otherwise always be running flywheel
      // TODO: actually shooting should be based on robotPose in the alliance zone, so it doesnt
      // have
      // to be a button
      // TODO: add funnel command (separate command) & instant command to stop running the flywheel
    } else {
      hopperFloorSubsystem.setHopperFloorVelocity(0);
      shooterSubsystem.setDesiredTransitionVoltage(0);
    }
    hoodSubsystem.setDesiredAngle(
        new Rotation2d(Math.PI / 2)
            .minus(params.hoodAngle)); // this requires the hood's zero to be parallel to the ground

    turretSubsystem.setDesiredAngle(params.turretAngle);
  }

  // TODO figure out if we want a way to end this command
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    hopperFloorSubsystem.setHopperFloorVelocity(0);
    shooterSubsystem.setDesiredTransitionVoltage(0);
  }
}
