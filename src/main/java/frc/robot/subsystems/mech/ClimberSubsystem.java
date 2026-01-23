package frc.robot.subsystems.mech;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants;



public class ClimberSubsystem extends SubsystemBase { 
    
    public final TalonFX motor;

    private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PIDController pidController;

    private static final double kP = 0.0; //TODO: tune all of these
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double DEADBAND = 500; //TODO: in ticks, edit, get climberDeadband method to use in command

    public ClimberSubsystem(){
        motor = new TalonFX(Constants.OUTER_ARM_MOTOR_CAN_ID);
        motor.setNeutralMode(NeutralModeValue.Brake); 
        pidController = new PIDController(kP, kI, kD);
    }

    public void setMotorOutput(double output){
        motor.setControl(dutyCycleOut.withOutput(output));
    }

    public void moveArm(double desiredTicks){
        double error = desiredTicks - getCurrentTicks();
        if(Math.abs(error)> DEADBAND) {
            double output = pidController.calculate(error);
            setMotorOutput(output);
        } else{
            setMotorOutput(0);
        }
    }

    public double getCurrentTicks(){
        return motor.getPosition().getValueAsDouble() * Constants.KRAKEN_TICKS_PER_REV;
    }

    public double inchesToTicks(double desiredInches){
        return desiredInches * Constants.CLIMBER_TICKS_PER_INCH;
    }

    public double getMotorOutput(){
        return motor.get();
    }

    public void setActiveMode() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setBrakeMode() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }
}
