package frc.robot.commands.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;

public class TrajectoryCommand extends Command {
  private Pose2d[] poses;
  private TrajectoryConfig config;
  private ArrayList<Translation2d> translations;
  private Trajectory trajectory;
  private HolonomicDriveController controller;
  private Timer timer;
  private Drive drive;

  private final double kPx = 0;
  private final double kIx = 0;
  private final double kDx = 0;

  private final double kPy = 0;
  private final double kIy = 0;
  private final double kDy = 0;

  private final double kPr = 0;
  private final double kIr = 0;
  private final double kDr = 0;

  public TrajectoryCommand(Pose2d[] poses, Drive drive) {
    this.poses = poses;
    this.drive = drive;

    controller =
        new HolonomicDriveController( // all the PIDs need tuning
            new PIDController(kPx, kIx, kDx),
            new PIDController(kPy, kIy, kDy),
            new ProfiledPIDController(
                kPr,
                kIr,
                kDr,
                new TrapezoidProfile.Constraints(
                    TrajectoryConstants.MAX_VELOCITY / TrajectoryConstants.DRIVETRAIN_RADIUS,
                    TrajectoryConstants.MAX_ACCELERATION / TrajectoryConstants.DRIVETRAIN_RADIUS)));

    config =
        new TrajectoryConfig( // these values need tuning
            TrajectoryConstants.MAX_VELOCITY, TrajectoryConstants.MAX_ACCELERATION);

    timer = new Timer();

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    trajectory = generateTrajectory();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    Trajectory.State desiredPose = trajectory.sample(timer.get());
    Rotation2d desiredHeading = desiredPose.poseMeters.getRotation();
    ChassisSpeeds adjustedSpeeds =
        controller.calculate(drive.getPose(), desiredPose, desiredHeading);
    SwerveModuleState[] moduleStates = drive.getKinematics().toSwerveModuleStates(adjustedSpeeds);
    drive.setModuleStates(moduleStates);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  public Trajectory generateTrajectory() {
    translations = new ArrayList<Translation2d>();
    for (int i = 1; i < poses.length - 1; i++) { // excludes start and end points
      translations.add(poses[i].getTranslation());
    }

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            poses[0], translations, poses[poses.length - 1], config);

    return trajectory;
  }
}
