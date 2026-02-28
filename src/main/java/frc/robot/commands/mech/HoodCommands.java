package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.mech.HoodSubsystem;

public class HoodCommands {

  private HoodCommands() {}

  public static Command RetractHood(HoodSubsystem hoodSubsystem) {
    return new HoodRetractCommand(hoodSubsystem).withName("Retract Hood");
  }

  public static Command HomeHood(HoodSubsystem hoodSubsystem) {
    return new HoodHomingFindLimitSwitchCommand(hoodSubsystem)
        .andThen(new HoodHomingBackOffCommand(hoodSubsystem))
        .andThen(new HoodHomingFinalCommand(hoodSubsystem))
        .withName("Hood Home");
  }

  // The reason this command isn't an instant command is so we don't go under the trench if the hood
  // hasn't finished retracting (that would be very bad)
  public static class HoodRetractCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodRetractCommand(HoodSubsystem hoodSubsystem) {
      this.hoodSubsystem = hoodSubsystem;
      addRequirements(hoodSubsystem);
      setName("Retract Hood");
    }

    @Override
    public void initialize() {
      hoodSubsystem.setDesiredAngle(HoodConstants.RETRACTED_POSITION);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
      if (Math.abs(
              hoodSubsystem.getCurrentAngle().getDegrees()
                  - hoodSubsystem.getDesiredAngle().getDegrees())
          <= HoodConstants.HOOD_POSITION_DEADBAND_DEGREES) {
        return true;
      } else if (hoodSubsystem.isRetractedLimitSwitchPressed()) {
        hoodSubsystem.setDesiredAngle(hoodSubsystem.getCurrentAngle());
        return true;
      }
      return false;
    }
  }

  /**
   * Stage 1 of hood homing: move fast toward the retracted limit switch and rough-zero when
   * pressed.
   */
  private static class HoodHomingFindLimitSwitchCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodHomingFindLimitSwitchCommand(HoodSubsystem hoodSubsystem) {
      this.hoodSubsystem = hoodSubsystem;
      addRequirements(hoodSubsystem);
      setName("HoodHomingLimitSwitch");
    }

    @Override
    public void initialize() {
      hoodSubsystem.setHoodVoltage(HoodConstants.FAST_HOMING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
      return hoodSubsystem.isRetractedLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
      hoodSubsystem.zeroHood();
    }
  }

  /**
   * Stage 2 of hood homing: back off to just before retracted position (RETRACTED - 5°) and wait
   * until the hood reaches that position (limit switch still pressed).
   */
  private static class HoodHomingBackOffCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodHomingBackOffCommand(HoodSubsystem hoodSubsystem) {
      this.hoodSubsystem = hoodSubsystem;
      addRequirements(hoodSubsystem);
      setName("HoodHomingBackOffCommand");
    }

    @Override
    public void initialize() {
      // TODO: tune -- the logic for homing was slightly wrong before so make sure this makes it so
      // the limit switch doesn't remain triggered
      hoodSubsystem.setDesiredAngle(
          HoodConstants.RETRACTED_POSITION.minus(new Rotation2d(Math.toRadians(5))));
    }

    @Override
    public boolean isFinished() {
      return !hoodSubsystem
          .isRetractedLimitSwitchPressed(); // we want to fully back off the limit switch
    }
  }

  /**
   * Stage 3 of hood homing: move slowly toward the retracted limit switch, final-zero when pressed,
   * and hold position at RETRACTED - 1°.
   */
  private static class HoodHomingFinalCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodHomingFinalCommand(HoodSubsystem hoodSubsystem) {
      this.hoodSubsystem = hoodSubsystem;
      addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
      hoodSubsystem.setHoodVoltage(HoodConstants.SLOW_HOMING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
      return hoodSubsystem.isRetractedLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
      hoodSubsystem.zeroHood();
      hoodSubsystem.setDesiredAngle(
          HoodConstants.RETRACTED_POSITION.minus(new Rotation2d(Math.toRadians(1))));
    }
  }
}
