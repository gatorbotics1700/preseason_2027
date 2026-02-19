package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Pose2d;
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
import org.littletonrobotics.junction.Logger;

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

  private double lastVelocity = 0;

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
    target =
        new Translation3d(
            3.734, 0, 0.813); // TODO: get rid of this when we want to use actual field coords
    ShotParameters params =
        ShotCalculator.calculateShot(drivetrainPose.get(), drivetrainVelocity.get(), target);
    System.out.println("PARAMS SHOT SPEED: " + params.shotSpeed);

    Logger.recordOutput("ShootingCommand/validShot", params.shotSpeed != 0);
    Logger.recordOutput("ShootingCommand/shouldShoot", shooterSubsystem.getShouldShoot());

    // if should be shooting
    // set flywheel speed to the last non-zero flywheel speed
    // if params.shotSpeed == 0 , stop the transition
    // if params.shotSpeed != 0 and our current speed matches that desired speed, run the transition

    // if shouldnt be shooting
    // stop everything including flywheel
    if (params.shotSpeed != 0) { // if we have a valid shot
      lastVelocity =
          ShooterSubsystem.calculateFlywheelSpeed(
              params.shotSpeed); // update our last velocity / set our desired velocity
    }

    if (shooterSubsystem.getShouldShoot()) { // if we want to shoot
      System.out.println("WE WANT TO SHOOT");
      if (params.shotSpeed != 0) { // and if we have a valid shot
        System.out.println("VALID SHOT VALID SHOT");
        shooterSubsystem.setFlywheelVelocity(lastVelocity); // set velocity to our desired velocity
        hopperFloorSubsystem.setHopperFloorVelocity(HopperFloorSubsystem.HOPPER_FLOOR_SPEED);
        if (Math.abs(shooterSubsystem.getFlywheelVelocity() - lastVelocity)
            < ShooterSubsystem
                .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired
          // velocity
          System.out.println("SHOOTING SHOOTING SHOOTING");
          shooterSubsystem.setDesiredTransitionVoltage(ShooterSubsystem.TRANSITION_VOLTAGE);
        }
      } else { // if we dont have a valid shot
        System.out.println("INVALID SHOT INVALID SHOT");
        hopperFloorSubsystem.setHopperFloorVelocity(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
      }
    } else {
      System.out.println("WE DONT WANT TO SHOOT");
      shooterSubsystem.setFlywheelVelocity(0);
      shooterSubsystem.setFlywheelVoltage(0);
      shooterSubsystem.setDesiredTransitionVoltage(0);
      hopperFloorSubsystem.setHopperFloorVelocity(0);
    }

    // if (shooterSubsystem.getShouldShoot()) {
    //   // calculate angles and get the hood and turret to track

    //   // only want to run hopper if we're actively trying to shoot
    //   // TODO figure out if this is the logic we actually want
    //   if (params.shotSpeed != 0) {
    //     hopperFloorSubsystem.setHopperFloorVelocity(HopperFloorSubsystem.HOPPER_FLOOR_SPEED);
    //     shooterSubsystem.setFlywheelVelocity(
    //         ShooterSubsystem.calculateFlywheelSpeed(params.shotSpeed));
    //     if (Math.abs(
    //             shooterSubsystem.getFlywheelVelocity()
    //                 - ShooterSubsystem.calculateFlywheelSpeed(params.shotSpeed))
    //         < shooterSubsystem.FLYWHEEL_SPEED_DEADBAND) {
    //       shooterSubsystem.setDesiredTransitionVoltage(ShooterSubsystem.TRANSITION_VOLTAGE);
    //     }
    //   } else {
    //     shooterSubsystem.setDesiredTransitionVoltage(0);
    //     hopperFloorSubsystem.setHopperFloorVelocity(0);
    //   }

    //   // turretSubsystem.setDesiredAngle(params.turretAngle);
    //   // TODO add drivetrain angle things here instead of the turret angle for testing on sting

    //   // since angle calculations for the shot are ground-rleative

    //   // if we're actually trying to shoot, and the flywheel is up to speed, kick the balls into
    // the
    //   // shooter!
    //   // TODO: add a way to indicate whether we're actually shooting, if we are, run kicker, but
    //   // otherwise always be running flywheel
    //   // TODO: actually shooting should be based on robotPose in the alliance zone, so it doesnt
    //   // have
    //   // to be a button
    //   // TODO: add funnel command (separate command) & instant command to stop running the
    // flywheel
    // } else {
    //   hopperFloorSubsystem.setHopperFloorVelocity(0);
    //   shooterSubsystem.setDesiredTransitionVoltage(0);
    // }
    hoodSubsystem.setDesiredAngle(
        params.hoodAngle); // this requires the hood's zero to be vertical TODO: Check this!!

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
