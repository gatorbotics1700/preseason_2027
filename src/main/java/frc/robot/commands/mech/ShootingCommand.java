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

    Logger.recordOutput("Mech/ShotCalculator/target", target);
    Logger.recordOutput("Mech/ShotCalculator/drivetrainPose", drivetrainPose.get());
    Logger.recordOutput("Mech/ShotCalculator/drivetrainVelocity", drivetrainVelocity.get());
    Logger.recordOutput("Mech/ShotCalculator/validShot", params.shotSpeed != 0);
    Logger.recordOutput("Mech/ShotCalculator/shotSpeed", params.shotSpeed);
    Logger.recordOutput("Mech/ShotCalculator/hoodAngle", params.hoodAngle);
    Logger.recordOutput("Mech/ShotCalculator/turretAngle", params.turretAngle);

    // if should be shooting
    // set flywheel speed to the last non-zero flywheel speed
    // if params.shotSpeed == 0 , stop the transition
    // if params.shotSpeed != 0 and our current speed matches that desired speed, run the transition

    // if shouldnt be shooting
    // stop everything including flywheel //TODO: is this the logic we actually want? i dont think
    // we want to stop the flywheel
    if (params.shotSpeed != 0) { // if we have a valid shot
      lastVelocity =
          ShooterSubsystem.calculateFlywheelSpeed(
              params.shotSpeed); // update our last velocity / set our desired velocity
    }

    // TODO: get should shoot hasn't been working/setting properly -- we probably want the logic to
    // set better and automatically, but need to test further
    // reasoning for this boolean rather than scheduling the command and stopping it more frequently
    // -- we want this to be as automated as possible (so based on pose)
    // but the problem if it's only based on pose is that if we want to stop shooting for some
    // reason or don't want to funnel in the neutral zone, we would have to stop the command from
    // running
    // if we get a toggle working for shouldShoot, we can keep the command running, default to not
    // shooting if we're in the neutral zone, but the command is still updating pose
    // and default to shooting in our zone, but then the toggle can change that behavior
    // i (phoenix) can't currently think of a better way to constantly be scheduling this command
    // automatically, but if people have ideas that would be great!
    if (shooterSubsystem.getShouldShoot()) { // if we want to shoot
      System.out.println("WE WANT TO SHOOT");
      if (params.shotSpeed != 0) { // and if we have a valid shot
        double desiredFlywheelSpeed = ShooterSubsystem.calculateFlywheelSpeed(params.shotSpeed);
        System.out.println("VALID SHOT VALID SHOT");
        shooterSubsystem.setDesiredFlywheelVelocity(
            desiredFlywheelSpeed); // set velocity to our desired velocity
        hopperFloorSubsystem.setDesiredHopperFloorVelocity(
            HopperFloorConstants.HOPPER_FLOOR_VELOCITY);
        if (Math.abs(shooterSubsystem.getFlywheelVelocity() - desiredFlywheelSpeed)
            < ShooterConstants
                .FLYWHEEL_SPEED_DEADBAND) { // once flywheel is running close to our desired
          // velocity
          System.out.println("SHOOTING SHOOTING SHOOTING");
          shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
        }
      } else { // if we dont have a valid shot
        System.out.println("INVALID SHOT INVALID SHOT");
        hopperFloorSubsystem.setDesiredHopperFloorVelocity(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
      }
      // this requires the hood's zero to be vertical TODO: Check this!!
      hoodSubsystem.setDesiredAngle(params.hoodAngle);
      turretSubsystem.setDesiredAngle(params.turretAngle);
    } /*else {
        System.out.println("WE DONT WANT TO SHOOT");
        shooterSubsystem.setDesiredFlywheelVelocity(0);
        // shooterSubsystem.setFlywheelVoltage(0);
        shooterSubsystem.setDesiredTransitionVoltage(0);
        hopperFloorSubsystem.setDesiredHopperFloorVelocity(0);
      } */

    // TODO add drivetrain angle things here instead of the turret angle for testing on sting

    // if we're actually trying to shoot, and the flywheel is up to speed, kick the balls into
    // the shooter!
    // TODO: add a way to indicate whether we're actually shooting, if we are, run kicker, but
    // otherwise always be running flywheel
    // TODO: actually shooting should be based on robotPose in the alliance zone, so it doesnt
    // have to be a button
    // TODO: add funnel command (separate command) & instant command to stop running the
    // flywheel

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // the command only ends if another command requires the subsystems and it's interrupted, so set
  // these things in end instead of isFinished
  @Override
  public void end(boolean interrupted) {
    // shooterSubsystem.setDesiredFlywheelVelocity(0); //TODO: do we want to set the flywheel
    // velocity to 0? don't we want to keep the flywheel running unless we specifically turn it off?
    hopperFloorSubsystem.setDesiredHopperFloorVelocity(0);
    shooterSubsystem.setDesiredTransitionVoltage(0);
  }
}
