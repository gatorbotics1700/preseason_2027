package frc.robot.subsystems.mech;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mech.MechIOs.HoodIO;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {
  public static final Rotation2d RETRACTED_POSITION =
      new Rotation2d(Math.toRadians(0)); // TODO: find a real number

  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  private Rotation2d desiredAngle = RETRACTED_POSITION;
  private final double POSITION_DEADBAND_DEGREES = 1; // TODO: tune
  private static final double HOOD_SHAFT_REVS_PER_MECH_REV = 155 / 15.0;
  private static final double HOOD_GEARBOX_RATIO = 9.0;
  private double desiredSpeed;

  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    setHoodSpeed(desiredAngle);

    Logger.recordOutput("hood desired angle", desiredAngle.getDegrees());
    // Logger.recordOutput("hood motor output", hoodMotor.get());
    // Logger.recordOutput("hood current angle", getCurrentAngle().getDegrees());
    // Logger.recordOutput("hood current velocity", hoodMotor.getVelocity().getValueAsDouble());
    io.updateInputs(inputs);
    //  Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/Velocity", inputs.velocityRevsPerSec);
    Logger.recordOutput("Hood/position", inputs.positionRevs);
    Logger.recordOutput("Hood/DesiredSpeed", desiredSpeed);
  }

  public void setDesiredAngle(Rotation2d desiredAngle) {
    this.desiredAngle = desiredAngle;
  }

  public void setHoodSpeed(Rotation2d desiredAngle) {
    io.setSpeed(desiredAngle);
  }

  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees / 360.0 * HOOD_SHAFT_REVS_PER_MECH_REV * HOOD_GEARBOX_RATIO;
  }
}
