package frc.robot.subsystems.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  public static final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(20)); // TODO find a real number for this

  private final HoodIO io;
  private final HoodIOInputs inputs = new HoodIOInputs();

  private Rotation2d desiredAngle = RETRACTED_POSITION;
  private final double POSITION_DEADBAND_DEGREES = 1; // TODO: tune
  private final int HOOD_GEAR_RATIO = 3; // TODO find the real value

  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    double angleError = currentAngle().getDegrees() - desiredAngle.getDegrees();
    if (Math.abs(angleError) > POSITION_DEADBAND_DEGREES) {
      setHoodSpeed(
          0.2 * angleError); // TODO check if this should be -angleError or if I have it backwards
    }
  }

  public void setDesiredAngle(Rotation2d desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public void setHoodSpeed(double speed) {
    io.setSpeed(speed);
  }

  /** Current hood angle from motor position (in revs) and gear ratio. */
  public Rotation2d currentAngle() {
    double outputRevs = inputs.positionRevs / HOOD_GEAR_RATIO;
    return Rotation2d.fromRotations(outputRevs);
  }
}
