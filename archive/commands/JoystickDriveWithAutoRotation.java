public class JoystickDriveWithAutoRotation {
  /**
   * Field relative drive command that automatically uses joystickDriveAtAngle when a desired angle
   * is set, otherwise uses regular joystickDrive.
   */
  public static Command joystickDriveWithAutoRotation(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    // Create PID controller for automatic rotation control (only used when desiredAngle is set)
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP * 3,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY * 12, ANGLE_MAX_ACCELERATION * 12));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    Rotation2d[] lastDesiredAngle = {null};

    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Check if desired angle is set (must check continuously, not just once)
              Rotation2d desiredAngle = drive.getDesiredAngle();
              Logger.recordOutput(
                  "DriveCommands/DesiredAngleDegrees",
                  desiredAngle != null ? Math.toDegrees(desiredAngle.getRadians()) : Double.NaN);
              // System.out.println("got desired angle: " + drive.getDesiredAngle());
              double omega;

              if (desiredAngle != null) {
                boolean isDynamic = drive.isDesiredAngleDynamic();
                Rotation2d previousAngle = lastDesiredAngle[0];
                boolean goalChanged = false;

                if (previousAngle == null) {
                  // First time setting an angle - reset the controller
                  // System.out.println("previous angle being set");
                  goalChanged = true;
                } else {
                  // Check if the goal has changed
                  double angleDifference = Math.abs(previousAngle.minus(desiredAngle).getRadians());
                  if (angleDifference > Math.PI) {
                    angleDifference = 2 * Math.PI - angleDifference;
                  }
                  // For dynamic angles (supplier), always reset each cycle since goal changes
                  // continuously
                  // For static angles, only reset if goal changed significantly
                  if (isDynamic || angleDifference > 1e-6) {
                    goalChanged = true;
                  }
                }

                // If goal changed, reset the controller to update its internal state
                // This is necessary because ProfiledPIDController maintains internal goal state
                if (goalChanged) {
                  // Reset with current position and zero velocity
                  // For dynamic angles, this happens every cycle to track the changing goal
                  angleController.reset(drive.getRotation().getRadians(), 0.0);
                  lastDesiredAngle[0] = desiredAngle;
                }

                // Use PID controller to automatically rotate to desired angle
                omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(), desiredAngle.getRadians());
              } else {
                lastDesiredAngle[0] = null;

                // Use joystick input for rotation
                // Apply rotation deadband
                omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);

                // Scale by max angular speed
                omega = omega * drive.getMaxAngularSpeedRadPerSec();
              }

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              angleController.reset(drive.getRotation().getRadians(), 0.0);
              lastDesiredAngle[0] = null;
            })
        .withName("JoystickDriveWithAutoRotation");
  }
}
