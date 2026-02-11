import edu.wpi.first.wpilibj2.command.Command;

public class ExampleCommand extends Command {
  // include any subsystem requirements here
  private final Subsystem subsystem;

  // constructor
  public ExampleCommand(Subsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // run any methods that should only run once (set desired position, etc)
  }

  @Override
  public void execute() {
    // set anything that will change as the subsystem state updates here
  }

  @Override
  public boolean isFinished() {
    // return true if only using initialize
    // use conditional statements to figure out when command should end (is the subsystem at the
    // setpoint?)
    return true;
  }
}
