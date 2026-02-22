package frc.robot.commands.mech;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.mech.HoodSubsystem;

// TODO: periodic independent of shooter to determine desired position of the hood -- use pose
// tracking
public class HoodRetractCommand extends Command {
  private final HoodSubsystem hoodSubsystem;

  public HoodRetractCommand(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);
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
        <= hoodSubsystem.HOOD_POSITION_DEADBAND_DEGREES) {
      return true;
    } else if (hoodSubsystem.isRetractedLimitSwitchPressed()) {
      hoodSubsystem.setDesiredAngle(HoodConstants.RETRACTED_POSITION);
      return true;
    }
    return false;
  }
}
