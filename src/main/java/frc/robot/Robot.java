package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TestCommand;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer container;

    public Robot() {
    }

    @Override
    public void robotInit() {
        container = new RobotContainer();
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = container.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }   

    @Override
    public void teleopInit() {
        container.setDefaultTeleopCommand();

        // This makes sure that the autonomous stops running when teleop starts
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        // Leave empty - default command will handle teleop
    }

    @Override
    public void testInit(){
        CommandScheduler.getInstance().schedule(new TestCommand(container.getDrivetrainSubsystem(), new Pose2d(1,0,new Rotation2d(0))));
    }

    @Override
    public void testPeriodic(){

    }
}
