package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;

public class TurretSubsystem
    extends SubsystemBase { // TODO: clean out turn by a certain angle stuff

  public final TalonFX motor;
  private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final PIDController pidController;

  private static final double kP = 0.025;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private Translation2d target = new Translation2d(0, 0);

  private Supplier<Pose2d> robotPose;

  public TurretSubsystem(Supplier<Pose2d> robotPose) {
    motor = new TalonFX(Constants.TURRET_MOTOR_CAN_ID, TunerConstants.mechCANBus);
    pidController = new PIDController(kP, kI, kD);
    this.robotPose = robotPose;
  }

  public void periodic() {
    turnToAngle(getTargetTurretAngle(target));
  }

  public void setTargetPoint(Translation2d targetPoint) {
    target = targetPoint;
  }

  public void turnToAngle(Rotation2d desiredAngle) {
    Rotation2d currentAngle = getTurretAngle();
    Rotation2d error = desiredAngle.minus(currentAngle);
    error = new Rotation2d(MathUtil.inputModulus(error.getRadians(), -Math.PI, Math.PI));

    if (Math.abs(error.getDegrees()) > Constants.TURRET_DEADBAND) {
      double output = pidController.calculate(error.getDegrees());
      setSpeed(output);
    } else {
      setSpeed(0);
    }
  }

  public Rotation2d getTurretAngle() {
    double angleInDegrees =
        -((motor.getPosition().getValueAsDouble() * 360 / Constants.TURRET_GEAR_RATIO) % 360);
    return new Rotation2d(Math.toRadians(angleInDegrees));
  }

  public void setSpeed(double speed) {
    motor.setControl(dutyCycleOut.withOutput(speed));
  }

  public double angleToTicks(double degrees) {
    return ((degrees % 360) / 360) * Constants.TURRET_GEAR_RATIO;
  }

  public Rotation2d getTargetTurretAngle(Translation2d target) {
    Pose2d currentRobotPose = robotPose.get();
    double deltaY = target.getY() - currentRobotPose.getY();
    double deltaX = target.getX() - currentRobotPose.getX();
    System.out.println(currentRobotPose);
    Rotation2d angleToTarget = new Rotation2d(Math.atan2(deltaY, deltaX));
    Rotation2d turretAngle = angleToTarget.minus(currentRobotPose.getRotation());
    return turretAngle;
  }
}
