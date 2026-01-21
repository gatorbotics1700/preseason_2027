package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class IntakeSubsystem extends SubsystemBase{
    
    public final TalonFX intakeMotor;
    public final TalonFX extensionMotor;

    private PIDController pidController;
    private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    private double error;
    private double extensionVoltage;

    private final double kP = 0.0; //TODO: tune all of these
    private final double kI = 0.0;
    private final double kD = 0.0;

    public final double EXTENDED_POSITION = 90; // TODO: change
    public final double RETRACTED_POSITION = 0; // TODO: change
    public final double DEADBAND = 2;    

    public IntakeSubsystem(){
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);
        extensionMotor = new TalonFX(Constants.EXTENSION_MOTOR_CAN_ID);
        pidController = new PIDController(kP, kI, kD);
    }
    
    @Override
    public void periodic(){
    }

    public void setMotorVoltage(double voltage){
        intakeMotor.setVoltage(voltage);
    }

    public void setExtensionVoltage(double output){
        extensionMotor.setControl(dutyCycleOut.withOutput(output));
    }

    public double getPosition(){
        return extensionMotor.getPosition().getValueAsDouble() * 360 / Constants.HOOD_GEAR_RATIO; //TODO: check this conversion into degrees
    }

    public void extendIntake(boolean wantExtended) {
        if(wantExtended == true){
            error = EXTENDED_POSITION - getPosition();
            if(Math.abs(error)>DEADBAND){
                extensionVoltage = pidController.calculate(error);
                setExtensionVoltage(extensionVoltage);
            } else{
                setExtensionVoltage(0);
            }
        } else { 
            error = RETRACTED_POSITION - getPosition();
            if(Math.abs(error)>DEADBAND){
                extensionVoltage = pidController.calculate(error);
                setExtensionVoltage(extensionVoltage);
            } else{
                setExtensionVoltage(0);
            }
        }
    }

}
