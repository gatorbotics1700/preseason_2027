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

// TODO: add mech commands into auto stuff
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldCoordinates;
import frc.robot.Constants.HopperFloorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.DriveOverBumpCommand;
import frc.robot.commands.drive.DriveUnderTrenchCommand;
import frc.robot.commands.mech.HoodCommands;
import frc.robot.commands.mech.IntakeCommands;
import frc.robot.commands.mech.ShootingCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.mech.ClimberSubsystem;
import frc.robot.subsystems.mech.HoodSubsystem;
import frc.robot.subsystems.mech.HopperFloorSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.ShooterSubsystem;
import frc.robot.subsystems.mech.TurretSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CommandSimMacXboxController;
import frc.robot.util.GamePieceSimulation;
import frc.robot.util.MultiStepAutoChooser;
import frc.robot.util.RobotConfigLoader;
import frc.robot.util.ShotCalculator;
import frc.robot.util.ShotParameters;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final HoodSubsystem hoodSubsystem = new HoodSubsystem();
  private final HopperFloorSubsystem hopperFloorSubsystem = new HopperFloorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();

  private final GamePieceSimulation gamePieceSimulation = new GamePieceSimulation();
  private ShotParameters shotParameters; // TODO: do we need this?

  // Controllers
  private CommandXboxController controller = null; // port 0
  private CommandXboxController controller_two = null; // port 3

  // Dashboard inputs
  private final MultiStepAutoChooser multiStepAutoChooser;
  private Supplier<Pose2d> robotPose;
  private Supplier<ChassisSpeeds> chassisSpeeds;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
                    VisionConstants.CAMERA_0_NAME, VisionConstants.ROBOT_TO_CAMERA_0));
        // new VisionIOPhotonVision(
        // VisionConstants.CAMERA_1_NAME, VisionConstants.ROBOT_TO_CAMERA_1));
        // TODO bring the second camera back yayyy -anne
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
        DriverStation.silenceJoystickConnectionWarning(true);
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

        break;
    }

    robotPose =
        () -> {
          return drive.getPose();
        };
    chassisSpeeds =
        () -> {
          return drive.getChassisSpeeds();
        };

    // Set up auto routines with multi-step chooser
    multiStepAutoChooser =
        new MultiStepAutoChooser(
            intakeSubsystem,
            drive,
            climberSubsystem,
            hoodSubsystem,
            shooterSubsystem,
            turretSubsystem,
            hopperFloorSubsystem,
            robotPose,
            chassisSpeeds);

    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureDriverButtonBindings() {
    if (DriverStation.isJoystickConnected(0)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller = new CommandSimMacXboxController(0);
      } else {
        controller = new CommandXboxController(0);
      }

      // Default command, normal field-relative drive
      Trigger driverControl =
          new Trigger(
              () ->
                  Math.abs(controller.getLeftY()) > 0.1
                      || Math.abs(controller.getLeftX()) > 0.1
                      || Math.abs(controller.getRightX()) > 0.1);

      driverControl
          .whileTrue(
              DriveCommands.joystickDrive(
                  drive,
                  () -> modifyJoystickAxis(-controller.getLeftY()), // Changed to raw values
                  () -> modifyJoystickAxis(-controller.getLeftX()), // Changed to raw values
                  () -> modifyJoystickAxis(-controller.getRightX()))) // Changed to raw values
          .onFalse(DriveCommands.stopDriveCommand(drive));

      // drive over bump

      controller
          .a()
          .onTrue(
              new InstantCommand(
                  () -> {
                    try {
                      CommandScheduler.getInstance()
                          .schedule(
                              DriveOverBumpCommand.driveOverBump(drive, shooterSubsystem)
                                  .withName("DriveOverBump"));
                    } catch (Exception e) {
                      e.printStackTrace();
                    }
                  }));

      controller
          .x()
          .onTrue(
              new InstantCommand(
                  () -> {
                    try {
                      CommandScheduler.getInstance()
                          .schedule(
                              DriveUnderTrenchCommand.driveUnderTrench(drive, shooterSubsystem)
                                  .withName("DriveUnderTrench"));
                    } catch (Exception e) {
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
                                  new Rotation2d(Math.toRadians(180))));
                        } else {
                          drive.setPose(
                              new Pose2d(
                                  drive.getPose().getTranslation(),
                                  new Rotation2d(Math.toRadians(0))));
                        }
                      },
                      drive)
                  .ignoringDisable(true));

      // drive under trench
      /* controller
                .x()
                .onTrue(
                    new InstantCommand(
                        () -> {
                          try {
                            CommandScheduler.getInstance()
                                .schedule(
                                    HoodCommands.RetractHood(hoodSubsystem)
                                        .alongWith(new ClimbCommands.HomeClimber(climberSubsystem))
                                        .andThen(
                                            DriveUnderTrenchCommand.driveUnderTrench(
                                                drive, shooterSubsystem))
                                        // .andThen(
                                        //     new ShootingCommand(
                                        //         shooterSubsystem,
                                        //         hoodSubsystem,
                                        //         turretSubsystem,
                                        //         hopperFloorSubsystem,
                                        //         robotPose,
                                        //         chassisSpeeds))
                                        .withName("DriveUnderTrench"));
                          } catch (Exception e) {
                            e.printStackTrace();
                          }
                        }));
      */
      controller.y().onTrue(HoodCommands.RetractHood(hoodSubsystem));

      controller
          .leftBumper()
          .onTrue(
              new InstantCommand(
                  () -> hoodSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(60)))));

      controller
          .rightBumper()
          .onTrue(
              Commands.runOnce(
                  () -> {
                    drive.setSlowDrive();
                  },
                  drive));
    }
  }

  public void configureCodriverButtonBindings() {
    if (DriverStation.isJoystickConnected(3)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller_two = new CommandSimMacXboxController(3);
        // putting this here because it should only run when we're in sim!

      } else {
        controller_two = new CommandXboxController(3);
      }

      if (Constants.currentMode == Constants.Mode.SIM) {
        controller_two.y().onTrue(LineupCommand());
        controller_two
            .a()
            .onTrue(
                new InstantCommand(
                    () -> {
                      // Use current pose and chassis speeds at this instant so all values match.
                      Pose2d pose = drive.getPose();

                      ChassisSpeeds cs = drive.getChassisSpeeds();
                      ShotParameters params =
                          ShotCalculator.calculateShot(pose, cs, FieldCoordinates.BLUE_HUB);

                      gamePieceSimulation.launchFuelBall(
                          ShotCalculator.getFieldToShooter(pose, ShooterConstants.BOT_TO_SHOOTER),
                          cs,
                          drive.getRotation(),
                          params.shotSpeed,
                          params.turretAngle,
                          params.hoodAngle);
                    }));
        controller_two
            .b()
            .onTrue(
                AutoBuilder.pathfindToPose(
                        new Pose2d(1, FieldCoordinates.BLUE_HUB.getY(), new Rotation2d()),
                        new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
                    .andThen(
                        Commands.parallel(
                            AutoBuilder.pathfindToPose(
                                new Pose2d(3, FieldCoordinates.BLUE_HUB.getY(), new Rotation2d()),
                                new PathConstraints(
                                    0.75, 12, Math.toRadians(700), Math.toRadians(1000))),
                            Commands.waitSeconds(0.2)
                                .andThen(
                                    Commands.runOnce(
                                        () -> {
                                          // Use current pose and chassis speeds at this instant so
                                          // all
                                          // values
                                          // match.
                                          Pose2d pose = drive.getPose();

                                          ChassisSpeeds cs = drive.getChassisSpeeds();
                                          ShotParameters params =
                                              ShotCalculator.calculateShot(
                                                  pose, cs, FieldCoordinates.BLUE_HUB);

                                          gamePieceSimulation.launchFuelBall(
                                              ShotCalculator.getFieldToShooter(
                                                  pose, ShooterConstants.BOT_TO_SHOOTER),
                                              cs,
                                              pose.getRotation(),
                                              params.shotSpeed,
                                              params.turretAngle,
                                              params.hoodAngle);
                                        })))));
        controller_two
            .x()
            .onTrue(
                AutoBuilder.pathfindToPose(
                        new Pose2d(1, FieldCoordinates.BLUE_HUB.getY() + 1, new Rotation2d()),
                        new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
                    .andThen(
                        Commands.parallel(
                            AutoBuilder.pathfindToPose(
                                new Pose2d(
                                    1, FieldCoordinates.BLUE_HUB.getY() + 2, new Rotation2d()),
                                new PathConstraints(
                                    0.75, 12, Math.toRadians(700), Math.toRadians(1000))),
                            Commands.waitSeconds(0.2)
                                .andThen(
                                    Commands.runOnce(
                                        () -> {
                                          // Use current pose and chassis speeds at this instant so
                                          // all
                                          // values
                                          // match.
                                          Pose2d pose = drive.getPose();

                                          ChassisSpeeds cs = drive.getChassisSpeeds();
                                          ShotParameters params =
                                              ShotCalculator.calculateShot(
                                                  pose, cs, FieldCoordinates.BLUE_HUB);

                                          gamePieceSimulation.launchFuelBall(
                                              ShotCalculator.getFieldToShooter(
                                                  pose, ShooterConstants.BOT_TO_SHOOTER),
                                              cs,
                                              pose.getRotation(),
                                              params.shotSpeed,
                                              params.turretAngle,
                                              params.hoodAngle);
                                        })))));
        // controller_two
        //     .x()
        //     .onTrue(
        //         AutoBuilder.pathfindToPose(
        //                 new Pose2d(16.516, 4.281, new Rotation2d()),
        //                 new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
        //             .andThen(
        //                 Commands.runOnce(
        //                     () -> {
        //                       // Use current pose and chassis speeds at this instant so
        //                       // all
        //                       // values
        //                       // match.
        //                       System.out.println("SHOOTING FROM RED TEST POSE");
        //                       Pose2d pose = drive.getPose();

        //                       ChassisSpeeds cs = drive.getChassisSpeeds();
        //                       ShotParameters params =
        //                           ShotCalculator.calculateShot(pose, cs,
        // FieldCoordinates.RED_HUB);

        //                       gamePieceSimulation.launchFuelBall(
        //                           ShotCalculator.getFieldToShooter(
        //                               pose, ShooterConstants.BOT_TO_SHOOTER),
        //                           cs,
        //                           pose.getRotation(),
        //                           params.shotSpeed,
        //                           params.turretAngle,
        //                           params.hoodAngle);
        //                     })));

        // controller_two
        //     .a()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> {
        //               shooterSubsystem.toggleShouldShoot();
        //               System.out.println("BUTTON AAAAAAAAA");
        //             }));

        // controller_two
        //     .x()
        //     .onTrue(
        //         new ShootingCommand(
        //             shooterSubsystem,
        //             hoodSubsystem,
        //             turretSubsystem,
        //             transitionSubsystem,
        //             robotPose,
        //             chassisSpeeds));
      } else {
        // TODO INTAKE TESTING BUTTONS - uncomment for use

        // controller_two
        //     .a()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> {
        //               intakeSubsystem.setDesiredAngle(IntakeConstants.EXTENDED_POSITION);
        //             }));

        // controller_two
        //     .b()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> {
        //               intakeSubsystem.setDesiredAngle(IntakeConstants.RETRACTED_POSITION);
        //             }));

        controller_two
            .a()
            .onTrue(
                new InstantCommand(
                    () ->
                        CommandScheduler.getInstance()
                            .schedule(IntakeCommands.RunIntake(intakeSubsystem))));

        controller_two
            .y()
            .onTrue(
                new InstantCommand(
                    () ->
                        CommandScheduler.getInstance()
                            .schedule(IntakeCommands.ToggleIntake(intakeSubsystem))));

        // controller_two.y().onTrue(new IntakeCommands.HomeIntakeDeploy(intakeSubsystem));

        // TODO TURRET TESTING BUTTONS - uncomment for use

        // controller_two
        //     .x()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> {
        //               turretSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(125)));
        //             }));

        // controller_two.y().onTrue(new TurretHomingCommand(turretSubsystem));

        // controller_two
        //     .a()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> {
        //               turretSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(-190)));
        //             }));

        // controller_two
        //     .b()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> {
        //               turretSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(200)));
        //             }));

        // TODO CLIMBER TESTING BUTTONS - uncomment for use
        // controller_two.x().onTrue(new ClimbCommands.HomeClimber(climberSubsystem));

        // controller_two.y().onTrue(ClimbCommands.RetractClimber(climberSubsystem));

        // controller_two.a().onTrue(ClimbCommands.ExtendClimber(climberSubsystem));

        // TODO HOOD TESTING BUTTONS - uncomment for use

        // controller_two
        // .rightBumper()
        // .onTrue(
        // new InstantCommand(
        // () -> {
        // hoodSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(70.0)));
        // }));

        // controller_two
        // .leftBumper()
        // .onTrue(
        // new InstantCommand(
        // () -> {
        // hoodSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(77.0)));
        // }));
        // controller_two
        // .y()
        // .onTrue(
        // HoodCommands.HomeHood(
        // hoodSubsystem)); // TODO: test and make sure this still works (calling it
        // // differently)
        // controller_two.a().onTrue(RunMechWheels());
        // controller_two.b().onTrue(MechStop());

        controller_two
            .b()
            .onTrue(
                new ShootingCommand(
                    shooterSubsystem,
                    hoodSubsystem,
                    turretSubsystem,
                    hopperFloorSubsystem,
                    robotPose,
                    chassisSpeeds));

        // controller_two
        //     .a()
        //     .onTrue(
        //         new InstantCommand(
        //             () ->
        //                 hopperFloorSubsystem.setDesiredHopperFloorVoltage(
        //                     HopperFloorConstants.HOPPER_FLOOR_VOLTAGE)));

        controller_two
            .x()
            .onTrue(
                new InstantCommand(
                    () ->
                        CommandScheduler.getInstance()
                            .schedule(
                                MechStop(
                                    turretSubsystem,
                                    shooterSubsystem,
                                    hopperFloorSubsystem,
                                    hoodSubsystem,
                                    intakeSubsystem))));
        // controller_two
        //     .y()
        //     .onTrue(new InstantCommand(() -> shooterSubsystem.setDesiredRotorVelocity(60)));

        // controller_two
        //     .y()
        //     .onTrue(
        //         new InstantCommand(
        //             () -> {
        //               hoodSubsystem.setDesiredAngle(new Rotation2d(Units.degreesToRadians(25)));
        //               shooterSubsystem.setDesiredFlywheelVelocity(90);
        //               shooterSubsystem.setDesiredTransitionVoltage(
        //                   ShooterConstants.TRANSITION_VOLTAGE);
        //             }));

        controller_two
            .rightBumper()
            .onTrue(new InstantCommand(() -> shooterSubsystem.toggleShouldShoot()));

        controller_two.leftBumper().onTrue(LineupCommand());
        // controller_two
        //     .leftBumper()
        //     .onTrue(
        //         new InstantCommand(
        //             () ->
        //                 shooterSubsystem.setDesiredTransitionVoltage(
        //                     ShooterConstants.TRANSITION_VOLTAGE)));

        // controller_two.b().onTrue(LineupCommand());
      }
    }
  }

  public void configureSysIdButtons() {
    if (DriverStation.isJoystickConnected(3)) {
      if (Constants.currentMode == Constants.Mode.SIM
          && System.getProperty("os.name").contains("Mac")) {
        controller_two = new CommandSimMacXboxController(3);
        // putting this here because it should only run when we're in sim!

      } else {
        controller_two = new CommandXboxController(3);
      }

      // TODO: drivetrain sysid buttons -- uncomment for use
      //  controller_two.a().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      //  controller_two.b().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      //  controller_two.x().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // controller_two.y().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

      // TODO: hood drivetrain sysid buttons -- uncomment for use
      // controller.x().whileTrue(hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // controller.y().whileTrue(hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // controller.a().whileTrue(hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // controller.b().whileTrue(hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // TODO: intake
      controller_two.x().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      controller_two.y().whileTrue(intakeSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      controller_two
          .a()
          .whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      controller_two
          .b()
          .whileTrue(intakeSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

      controller_two
          .rightBumper()
          .onTrue(
              new InstantCommand(() -> intakeSubsystem.zeroIntakeDeploy()).ignoringDisable(true));

      // TODO: shooter
      // controller_two.x().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      // controller_two.y().whileTrue(shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // controller_two
      //     .a()
      //     .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // controller_two
      //     .b()
      //     .whileTrue(shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    }
  }

  public Command getAutonomousCommand() {
    try {
      return multiStepAutoChooser.getAutonomousCommand();
    } catch (Exception ioe) {
      System.out.println("bad io error");
      return Commands.none();
    }
  }

  public Optional<Pose2d> getAutoStartPose() {
    return multiStepAutoChooser.getAutoStartPose();
  }

  public Drive getDriveSubsystem() {
    return drive;
  }

  private double deadband(double value, double deadband) {
    // If controller reads very tiny value close to zero, we don't want to make the
    // robot think it has to move
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

  public void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverButtonBindings();
    configureCodriverButtonBindings();
  }

  public void teleopInit() {
    if (RobotConfigLoader.getSerialNumber().equals(RobotConfigLoader.NILE_SERIAL)) {
      drive.enableTargetPointFacing();
    }
    configureButtonBindings();
  }

  public void periodic() {
    if (Constants.currentMode == Constants.simMode) {
      gamePieceSimulation.updateBalls();
    }

    multiStepAutoChooser.updateChooserOptions();

    // Print path name to console me thinks
    // String selectedPathName = multiStepAutoChooser.getSelectedPathName();
    System.out.flush(); // Ensure output appears immediately

    // Log command scheduler status
    Logger.recordOutput("Commands/SchedulerActive", true);
    Logger.recordOutput("Commands/LogTime", System.currentTimeMillis());

    // Log command information with names
    Command driveCmd = drive.getCurrentCommand();

    Logger.recordOutput("Commands/DriveCommand", driveCmd != null ? driveCmd.getName() : "None");

    // Log if commands are running
    Logger.recordOutput("Commands/DriveCommandActive", driveCmd != null);
    Logger.recordOutput(
        "Commands/DriveToFuelActive",
        driveCmd != null ? driveCmd.getName().equals("DriveToFuel") : false);

    Logger.recordOutput("DriveToFuel/Fuel", vision.getFuelPose(drive.getPose()));
    Logger.recordOutput("Odometry/Fuel", vision.getFuelPose(drive.getPose()));

    // shotParameters =
    // ShotCalculator.calculateShot(
    // drive.getPose(), drive.getChassisSpeeds(), Constants.BLUE_HUB, 10);\
  }

  // TODO: this command doesn't work -- need to fix (test this by commenting out
  // the way we do this
  // manually in Robot.java)
  public Command MechStop(
      TurretSubsystem turretSubsystem,
      ShooterSubsystem shooterSubsystem,
      HopperFloorSubsystem hopperFloorSubsystem,
      HoodSubsystem hoodSubsystem,
      IntakeSubsystem intakeSubsystem) {
    return new InstantCommand(
            () -> {
              turretSubsystem.setDesiredAngle(turretSubsystem.getCurrentAngle());
              shooterSubsystem.setDesiredRotorVelocity(0);
              hopperFloorSubsystem.setDesiredHopperFloorVoltage(0);
              shooterSubsystem.setDesiredTransitionVoltage(0);
              hoodSubsystem.setDesiredAngle(hoodSubsystem.getCurrentAngle());
              hoodSubsystem.setHoodVoltage(0);
              // TODO add climber
            },
            turretSubsystem,
            shooterSubsystem,
            hopperFloorSubsystem,
            hoodSubsystem,
            intakeSubsystem
            // )
            )
        .alongWith(IntakeCommands.StopIntake(intakeSubsystem));
  }

  public Command LineupCommand() {
    return AutoBuilder.pathfindToPose(
            // new Pose2d(3.037, 3.6, new Rotation2d()),
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? new Pose2d(13.3, 7.2, new Rotation2d(-116))
                : new Pose2d(
                    FieldCoordinates.FIELD_CENTER.getX()
                        - Math.abs(FieldCoordinates.FIELD_CENTER.getX() - 13.3),
                    7.2,
                    new Rotation2d(-116 - 90)),
            new PathConstraints(4, 12, Math.toRadians(700), Math.toRadians(1000)))
        .andThen(
            new InstantCommand(
                () -> {
                  shooterSubsystem.setDesiredRotorVelocity(62.2);
                  hoodSubsystem.setDesiredAngle(new Rotation2d(Math.toRadians(64)));
                  shooterSubsystem.setDesiredTransitionVoltage(ShooterConstants.TRANSITION_VOLTAGE);
                  hopperFloorSubsystem.setDesiredHopperFloorVoltage(
                      HopperFloorConstants.HOPPER_FLOOR_VOLTAGE);
                }));
  }

  public Command RunShooterWheels() {
    return new InstantCommand(
        () -> {
          shooterSubsystem.setDesiredRotorVelocity(70);
          shooterSubsystem.setDesiredTransitionVoltage(12);
        });
  }

  public Command RunMechWheels() {
    return new InstantCommand(
            () -> {
              shooterSubsystem.setDesiredRotorVelocity(70);
              shooterSubsystem.setDesiredTransitionVoltage(12);
              // transitionSubsystem.setHopperFloorVelocity(transitionSubsystem.HOPPER_FLOOR_SPEED);
              // TODO uncomment???
            })
        .alongWith(IntakeCommands.RunIntake(intakeSubsystem));
  }

  public Command HomeMechanisms() { // TODO: add any other homing commands with alongWith
    return HoodCommands.HomeHood(hoodSubsystem)
        // .alongWith(
        //     new ClimbCommands.HomeClimber(
        //         climberSubsystem)); // .alongWith(new TurretHomingCommand(turretSubsystem));
        .alongWith(new IntakeCommands.HomeIntakeDeploy(intakeSubsystem));
  }

  public TurretSubsystem getTurretSubsystem() {
    return turretSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public HoodSubsystem getHoodSubsystem() {
    return hoodSubsystem;
  }

  public HopperFloorSubsystem getHopperFloorSubsystem() {
    return hopperFloorSubsystem;
  }
}
