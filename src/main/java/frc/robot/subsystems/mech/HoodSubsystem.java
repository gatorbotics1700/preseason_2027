package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;

public class HoodSubsystem extends SubsystemBase {
  public final TalonFX hoodMotor;
  private boolean isTargetting;
  private double hoodVoltage;
  private PIDController pidController;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private Translation2d shootingToPosition;

  private static final double kP = 0.001; // TODO: tune all of these
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private final double POSITION_DEADBAND_TICKS = degreesToTicks(1.0); // TODO: tune

  private Supplier<Pose2d> robotPose;

  public HoodSubsystem(Supplier<Pose2d> robotPose) {
    hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    hoodMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    pidController = new PIDController(kP, kI, kD);
  }

  public void periodic() {
    if (isTargetting) {
      turnToPosition(getHoodTargetPosition(shootingToPosition));
    }
  }

  public void setShootingToPosition(
      Translation2d shootingToPosition) { // this is for once we start testing targetting
    this.shootingToPosition = shootingToPosition;
  }

  public void setHoodSpeed(double speed) {
    hoodMotor.setControl(dutyCycleOut.withOutput(speed));
    System.out.println("SETTING HOOD SPEED");
  }

  public double getHoodOutput() {
    return hoodMotor.get();
  }

  public double getHoodPositionTicks() {
    return hoodMotor.getPosition().getValueAsDouble(); // TODO: check this conversion into degrees
  }

  public double getHoodTargetPosition(Translation2d shootingToPosition) {
    Pose2d currentRobotPose = robotPose.get();
    double deltaY = Math.abs(shootingToPosition.getY() - currentRobotPose.getY());
    double deltaX = Math.abs(shootingToPosition.getX() - currentRobotPose.getX());
    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    // replace with InterpolatingDoubleTreeMap getter to find degrees using distance
    return degreesToTicks(45);
  }

  public double getDistanceFromFuelTarget() {
    return 10;
  }

  public void turnToPosition(double targetPositionTicks) {
    double currentPositionTicks = getHoodPositionTicks();
    double errorTicks = targetPositionTicks - currentPositionTicks;
    System.out.println("HOOD POSITION ERROR:" + errorTicks);

    // Deadband: stop motor when close enough to target
    if (Math.abs(errorTicks) < POSITION_DEADBAND_TICKS) {
      setHoodSpeed(0);
      System.out.println("HOOD AT POSITION. STOPPING");
    } else {
      hoodVoltage = pidController.calculate(currentPositionTicks, targetPositionTicks);
      setHoodSpeed(hoodVoltage);
      System.out.println("MOVING HOOD!!");
    }
  }

  public void setIsTargetting(boolean isTargetting) {
    this.isTargetting = isTargetting;
  }

  public double degreesToTicks(double degrees) {
    return degrees
        / 360
        * Constants.HOOD_SHAFT_REVS_PER_MECH_REV
        * Constants.HOOD_GEAR_RATIO
        * Constants.KRAKEN_TICKS_PER_REV;
  }

  public double ticksToDegrees(double ticks) {
    return ticks
        / Constants.KRAKEN_TICKS_PER_REV
        / Constants.HOOD_GEAR_RATIO
        / Constants.HOOD_SHAFT_REVS_PER_MECH_REV
        * 360;
  }
}
