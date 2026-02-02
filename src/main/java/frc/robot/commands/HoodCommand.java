package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.HoodSubsystem;

public class HoodCommand extends Command {
  private HoodSubsystem hoodSubsystem;
  private double startTime;
  private double targetPosition;
  private boolean isTargetting;

  public HoodCommand(HoodSubsystem hoodSubsystem, boolean isTargetting, double targetPosition) {
    this.hoodSubsystem = hoodSubsystem;
    this.targetPosition = targetPosition;
    this.isTargetting = isTargetting;
    addRequirements(hoodSubsystem);
  }

  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    hoodSubsystem.setIsTargetting(isTargetting);
    // hoodSubsystem.setShootingToPosition(shootingToPosition); this is for once we start testing
    // targetting
    System.out.println("HOOD STARTING POSITION REVS:" + hoodSubsystem.getHoodPositionMotorRevs());
    System.out.println(
        "HOOD STARTING DEGREES:"
            + hoodSubsystem.motorRevsToDegrees(hoodSubsystem.getHoodPositionMotorRevs()));
  }

  @Override
  public void execute() {
    // if (!isTargetting) {
    hoodSubsystem.turnToPosition(hoodSubsystem.degreesToMotorRevs(targetPosition));
    System.out.println("USING HOOD COMMAND");
    // }
    SmartDashboard.putNumber("Hood target position (degrees)", targetPosition);
  }

  @Override
  public boolean isFinished() {
    if (hoodSubsystem.getHoodOutput() == 0) {
      return true;
    }
    return false;
  }
}
