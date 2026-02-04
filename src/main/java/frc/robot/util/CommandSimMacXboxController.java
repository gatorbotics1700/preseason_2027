package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// In sim on mac, the buttons are jumbled. Here, we reverse the jumbling.
public class CommandSimMacXboxController extends CommandXboxController {

  public CommandSimMacXboxController(Integer port) {
    super(port);
  }

  @Override
  public Trigger a() {
    return super.button(1);
  }

  @Override
  public Trigger b() {
    return super.button(2);
  }

  @Override
  public Trigger x() {
    return super.button(4);
  }

  @Override
  public Trigger y() {
    return super.button(5);
  }

  @Override
  public Trigger back() {
    return super.button(11);
  }

  @Override
  public Trigger start() {
    return super.button(12);
  }

  @Override
  public Trigger rightBumper() {
    return super.button(8);
  }

  @Override
  public Trigger rightTrigger() {
    return super.axisGreaterThan(4, -.99);
  }

  @Override
  public Trigger leftBumper() {
    return super.button(7);
  }

  @Override
  public Trigger leftTrigger() {
    return super.axisGreaterThan(5, -.99);
  }

  @Override
  public double getLeftX() {
    return super.getRawAxis(0);
  }

  @Override
  public double getLeftY() {
    return super.getRawAxis(1);
  }

  @Override
  public double getRightX() {
    return super.getRawAxis(2);
  }

  @Override
  public double getRightY() {
    return super.getRawAxis(3);
  }
}
