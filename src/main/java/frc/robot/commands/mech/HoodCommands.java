package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.mech.HoodSubsystem;

public class HoodCommands {

  private HoodCommands() {}

  public static Command RetractHood(HoodSubsystem hoodSubsystem) {
    return new HoodRetractCommand(hoodSubsystem).withName("Retract Hood");
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
      } else if (hoodSubsystem.isCurrentLimitReached()) {
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
  public static class HoodHomingCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodHomingCommand(HoodSubsystem hoodSubsystem) {
      this.hoodSubsystem = hoodSubsystem;
      addRequirements(hoodSubsystem);
      setName("HoodHomingLimitSwitch");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      hoodSubsystem.setHoodVoltage(HoodConstants.FAST_HOMING_VOLTAGE);
    }

    @Override
    public boolean isFinished() {
      return hoodSubsystem.isCurrentLimitReached();
    }

    @Override
    public void end(boolean interrupted) {
      hoodSubsystem.zeroHood();
    }
  }
}
