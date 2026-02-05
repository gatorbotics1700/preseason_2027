// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

// import frc.robot.commands.AutoDriveCommand;
// import frc.robot.commands.TeleopDriveCommand;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.MechIOs.HoodIO;
import frc.robot.subsystems.mech.MechIOs.HoodIOSim;
import frc.robot.subsystems.mech.MechIOs.HoodIOTalonFX;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.GamePieceSimulation;
import frc.robot.util.MultiStepAutoChooser;
import frc.robot.util.RobotConfigLoader;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final HoodSubsystem hoodSubsystem;
  private final HopperFloorSubsystem transitionSubsystem = new HopperFloorSubsystem();
  private final GamePieceSimulation gamePieceSimulation = new GamePieceSimulation();
  // private final TurretSubsystem turretSubsystem;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller_two = new CommandXboxController(3);

  private final GenericHID buttonBoard1A = new GenericHID(1);
  private final GenericHID buttonBoard1B = new GenericHID(2);

  // Dashboard inputs
  private final MultiStepAutoChooser multiStepAutoChooser;

  // Button Bindings

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // new Trigger(controller_two::getAButtonPressed)
    //     .onTrue(
    //         new ShooterCommand(shooterSubsystem, Constants.FLYWHEEL_SHOOTING_VOLTAGE)
    //             .alongWith(new WaitCommand(3.0))
    //             .andThen(
    //                 new HopperFloorCommand(
    //                     transitionSubsystem, Constants.KICKER_SHOOTING_VOLTAGE, 0)));
    // new Trigger(controller_two::getBButtonPressed)
    //     .onTrue(
    //         new TransitionCommand(transitionSubsystem, 0, 0)
    //             .alongWith(new ShooterCommand(shooterSubsystem, 0)));

    // Named Commands
    NamedCommands.registerCommand(
        "Shooter Command",
        new InstantCommand(
            () -> {
              CommandScheduler.getInstance().schedule(Commands.none());
            }));
    NamedCommands.registerCommand(
        "Climb Command",
        new InstantCommand(
            () -> {
              CommandScheduler.getInstance().schedule(Commands.none());
            }));

    NamedCommands.registerCommand(
        "Intake Command",
        new InstantCommand(
            () -> {
              CommandScheduler.getInstance().schedule(Commands.none());
            }));

    NamedCommands.registerCommand(
        "Stop Kicker Command",
        new InstantCommand(
            () -> {
              CommandScheduler.getInstance().schedule(Commands.none());
            }));

    // Set up robot depending on mode
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_0_NAME, VisionConstants.ROBOT_TO_CAMERA_0),
                new VisionIOPhotonVision(
                    VisionConstants.CAMERA_1_NAME, VisionConstants.ROBOT_TO_CAMERA_1));
        hoodSubsystem = new HoodSubsystem(new HoodIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                (pose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_0_NAME,
                    VisionConstants.ROBOT_TO_CAMERA_0,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.CAMERA_1_NAME,
                    VisionConstants.ROBOT_TO_CAMERA_1,
                    drive::getPose));
        hoodSubsystem = new HoodSubsystem(new HoodIOSim());
        break;

      default: // TODO: should the default be real as a safety for matches? to be discussed
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision = new Vision(drive);
        hoodSubsystem = new HoodSubsystem(new HoodIO() {});
        break;
    }

    Supplier<Pose2d> robotPose =
        () -> {
          return drive.getPose();
        };
    // turretSubsystem = new TurretSubsystem(robotPose);

    // mech buttons
    // new Trigger(controller_two::getXButtonPressed)
    //     .onTrue(new HoodCommand(hoodSubsystem, false, 15));
    // new Trigger(controller_two::getYButtonPressed).onTrue(new HoodCommand(hoodSubsystem, false,
    // 0));

    // Set up auto routines with multi-step chooser
    multiStepAutoChooser = new MultiStepAutoChooser();

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    // Default command, normal field-relative drive
    // Uses joystickDriveAtAngle when desiredAngle is set, otherwise uses joystickDrive
    Trigger driverControl =
        new Trigger(
            () ->
                Math.abs(controller.getLeftY()) > 0.1
                    || Math.abs(controller.getLeftX()) > 0.1
                    || Math.abs(controller.getRightX()) > 0.1);
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      driverControl
          .whileTrue(
              DriveCommands.joystickDrive(
                  drive,
                  () -> modifyJoystickAxis(controller.getLeftY()), // Changed to raw values
                  () -> modifyJoystickAxis(controller.getLeftX()), // Changed to raw values
                  () -> modifyJoystickAxis(-controller.getRightX()))) // Changed to raw values
          .onFalse(DriveCommands.stopDriveCommand(drive));
    } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
      driverControl
          .whileTrue(
              DriveCommands.joystickDrive(
                  drive,
                  () -> modifyJoystickAxis(-controller.getLeftY()), // Changed to raw values
                  () -> modifyJoystickAxis(-controller.getLeftX()), // Changed to raw values
                  () -> modifyJoystickAxis(-controller.getRightX()))) // Changed to raw values
          .onFalse(DriveCommands.stopDriveCommand(drive));
    }

    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  try {
                    CommandScheduler.getInstance()
                        .schedule(DriveOverBumpCommand.driveOverBump(drive));
                  } catch (Exception e) {
                    // System.out.println("CATCHING EXCEPTION DAHHHHHHHHHHHHHHHHHH");
                    e.printStackTrace();
                  }
                }));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                new Rotation2d(Math.toRadians(0))));
                      } else {
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                new Rotation2d(Math.toRadians(0))));
                      }
                    },
                    drive)
                .ignoringDisable(true));
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  try {
                    CommandScheduler.getInstance()
                        .schedule(DriveUnderTrenchCommand.driveUnderTrench(drive, hoodSubsystem));
                  } catch (Exception e) {
                    e.printStackTrace();
                  }
                }));

    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.disableTargetPointFacing();
                }));
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  try {
                    CommandScheduler.getInstance()
                        .schedule(DriveOverBumpCommand.driveOverBump(drive));
                  } catch (Exception e) {
                    e.printStackTrace();
                  }
                }));

    // controller_two
    //     .back()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               drive.setPose(new Pose2d(4, 2, new Rotation2d(Math.toRadians(0))));
    //             },
    //             drive));

    // controller_two
    //     .x()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () -> {
    //                   vision.takePicture();
    //                 })
    //             .ignoringDisable(true));

    controller
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.setSlowDrive();
                },
                drive));

    // controller_two
    //     .leftBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               Logger.recordOutput(
    //                   "Robot/TargetPose",
    //                   LineupCommand.getLineupTagPose(
    //                       DriverStation.getAlliance().orElse(Alliance.Blue),
    //                       ReefSide.LeftSubstation));
    //               Logger.recordOutput("Robot/LineupSide", "LeftSubstation");
    //               Logger.recordOutput("Robot/IsLeftSide", false);
    //               CommandScheduler.getInstance()
    //                   .schedule(LineupCommand.Lineup(ReefSide.LeftSubstation, YOffset.Center));
    //             }));
    // controller_two
    //     .rightBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               Logger.recordOutput(
    //                   "Robot/TargetPose",
    //                   LineupCommand.getLineupTagPose(
    //                       DriverStation.getAlliance().orElse(Alliance.Blue),
    //                       ReefSide.RightSubstation));
    //               Logger.recordOutput("Robot/LineupSide", "RightSubstation");
    //               Logger.recordOutput("Robot/IsLeftSide", false);
    //               CommandScheduler.getInstance()
    //                   .schedule(LineupCommand.Lineup(ReefSide.RightSubstation, YOffset.Center));
    //             }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    try {
      return multiStepAutoChooser.getAutonomousCommand();
    } catch (Exception ioe) {
      System.out.println("bad io error");
      return Commands.none();
    }
  }

  public Drive getDriveSubsystem() {
    return drive;
  }

  private double deadband(double value, double deadband) {
    // If controller reads very tiny value close to zero, we don't want to make the robot think it
    // has to move
    // Without deadband, robot will think it has to move, and then it will go crazy
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0; // Return 0 if absolute value is within desired margin of error
    }
  }

  private double modifyJoystickAxis(double value) {
    // Deadband
    value = deadband(value, 0.025);

    // Square the axis
    value =
        Math.copySign(
            RobotConfigLoader.getDouble("container.joystick_scale_factor") * Math.pow(value, 2),
            value);

    if (drive.getSlowDrive()) {
      return 0.5 * value;
    }

    return value;
  }

  public void teleopInit() {
    drive.enableTargetPointFacing();
  }

  /**
   * Periodic method to log button states and other robot information. Call this from
   * Robot.teleopPeriodic() and Robot.autonomousPeriodic().
   */
  public void periodic() {
    gamePieceSimulation.updateBalls();
    // Update multi-step auto chooser options (reads choosers to keep them active)
    multiStepAutoChooser.updateChooserOptions();

    // Print selected path name to console
    String selectedPathName = multiStepAutoChooser.getSelectedPathName();
    System.out.flush(); // Ensure output appears immediately

    // Log button states directly - much simpler!
    Logger.recordOutput("Buttons/Controller1/A", controller.a().getAsBoolean());
    Logger.recordOutput("Buttons/Controller1/B", controller.b().getAsBoolean());
    Logger.recordOutput("Buttons/Controller1/X", controller.x().getAsBoolean());
    Logger.recordOutput("Buttons/Controller1/Y", controller.y().getAsBoolean());

    // Logger.recordOutput("Buttons/Controller2/A", controller_two.a().getAsBoolean());
    // Logger.recordOutput("Buttons/Controller2/B", controller_two.b().getAsBoolean());
    // Logger.recordOutput("Buttons/Controller2/X", controller_two.x().getAsBoolean());
    // Logger.recordOutput("Buttons/Controller2/Y", controller_two.y().getAsBoolean());

    // Log button board states
    for (int i = 1; i <= 6; i++) {
      Logger.recordOutput("Buttons/ButtonBoard1A/Button" + i, buttonBoard1A.getRawButton(i));
      Logger.recordOutput("Buttons/ButtonBoard1B/Button" + i, buttonBoard1B.getRawButton(i));
    }

    // Log command scheduler status
    Logger.recordOutput("Commands/SchedulerActive", true);
    Logger.recordOutput("Commands/LogTime", System.currentTimeMillis());

    // Log command information with names
    Command driveCmd = drive.getCurrentCommand();

    Logger.recordOutput("Commands/DriveCommand", driveCmd != null ? driveCmd.getName() : "None");

    // Log if commands are running
    Logger.recordOutput("Commands/DriveCommandActive", driveCmd != null);

    // controller_two
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               gamePieceSimulation.launchFuelBall(
    //                   new Translation3d(0, 0, 0),
    //                   10,
    //                   new Rotation2d(Math.toRadians(45)),
    //                   new Rotation2d(0));
    //             }));
  }
}
