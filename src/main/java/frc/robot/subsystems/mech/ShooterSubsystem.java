package frc.robot.subsystems.mech;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    public final TalonFX flywheelMotor;
    public final TalonFX hoodMotor;
    private boolean isTargetting;
    private double hoodVoltage;
    private PIDController pidController;
    private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    private static final double kP = 0.0; //TODO: tune all of these
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public ShooterSubsystem(){
        flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID);
        hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID);
        pidController = new PIDController(kP, kI, kD);
    }

    public void periodic(){
        if(isTargetting==true){
            turnToPosition(getTargetPosition());
        }
    }

    public void setFlywheelVoltage(double flywheelVoltage){
        flywheelMotor.setVoltage(flywheelVoltage);
    }

    public void setHoodSpeed(double speed){
        hoodMotor.setControl(dutyCycleOut.withOutput(speed));
    }

    public double getPosition(){
        return hoodMotor.getPosition().getValueAsDouble() * 360 / Constants.HOOD_GEAR_RATIO; //TODO: check this conversion into degrees
    }

    public double getTargetPosition(){
        return 45; // replace with InterpolatingDoubleTreeMap getter?
    }

    public void turnToPosition(double targetPosition){
        hoodVoltage = pidController.calculate(targetPosition - getPosition());
        setHoodSpeed(hoodVoltage);
    }

    public void setIsTargetting(boolean isTargetting){
        this.isTargetting = isTargetting;
    }

}
