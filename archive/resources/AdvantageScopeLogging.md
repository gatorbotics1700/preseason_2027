# Link to API: https://docs.advantagekit.org/javadoc/index.html

# Logging:
Logger.recordOutput("path/to/your/key", value);
NOTE
value:
value.getAsBoolean()
value.getAsInteger()
and so on for your type

# Automatic logging ex:
 @AutoLogOutput(key = "Robot/TargetPose")
  public Pose2d getTargetPose() {
    return targetPose;
  }
