package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

//things this subsystem needs to handle: 
//a TalonFX motor, and its DutyCycleOut, which you will use to set the motor speeds
//the position of the turret--methods and ratios you may find helpful for this: 
//the turret is on a 42:1 gear ratio, and you can get the motor position with .getPosition().getValueAsDouble()

public class TurretSubsystem extends SubsystemBase {
  //TODO create your variables here (hint: you should only need two)

  public TurretSubsystem() {
    //TODO initialize the motor (NOTE: the CANBus is in TunerConstants, NOT Constants.java)
  }

  @Override
  public void periodic() {

  }
  //TODO add methods for setting speed and accessing the TURRET position
}
