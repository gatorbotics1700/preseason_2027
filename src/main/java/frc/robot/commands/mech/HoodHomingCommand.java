package frc.robot.commands.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.mech.HoodSubsystem;

/**
 * Runs the hood fast toward retract until the retracted limit switch is pressed, then backs off and
 * slowly runs into the limit switch and then zeros position.
 */
public class HoodHomingCommand extends Command {
  private final HoodSubsystem hoodSubsystem;
  private boolean stageOneComplete;
  private boolean stageTwoComplete;
  private boolean stageThreeComplete;

  public HoodHomingCommand(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    stageOneComplete = false;
    stageTwoComplete = false;
    stageThreeComplete = false;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!stageOneComplete) {
      // Move fast towards limit switch and rough zero
      hoodSubsystem.setHoodVoltage(HoodConstants.FAST_HOMING_VOLTAGE);
      if (hoodSubsystem.isRetractedLimitSwitchPressed()) {
        hoodSubsystem.zeroHood();
        stageOneComplete = true;
      }
    } else if (!stageTwoComplete) {
      // Back off
      // TODO tune that angle
      hoodSubsystem.setDesiredAngle(
          HoodConstants.RETRACTED_POSITION.minus(new Rotation2d(Math.toRadians(5))));
      if (hoodSubsystem.isRetractedLimitSwitchPressed()) {
        stageTwoComplete = true;
      }
    } else {
      // Move slowly towards limit switch and final zero
      hoodSubsystem.setHoodVoltage(HoodConstants.SLOW_HOMING_VOLTAGE);
      if (hoodSubsystem.isRetractedLimitSwitchPressed()) {
        hoodSubsystem.zeroHood();
        hoodSubsystem.setDesiredAngle(
            HoodConstants.RETRACTED_POSITION.minus(new Rotation2d(Math.toRadians(1))));
        stageThreeComplete = true;
        System.out.println("ENDING");
      }
    }
    System.out.println("HOMING");
  }

  @Override
  public boolean isFinished() {
    return stageThreeComplete;
  }
}
