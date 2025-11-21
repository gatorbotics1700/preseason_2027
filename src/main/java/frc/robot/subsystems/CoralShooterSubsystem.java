// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralShooterSubsystem extends SubsystemBase {

  TalonFX talonRight;
  TalonFX talonLeft;
  DigitalInput limitSwitch;

  public CoralShooterSubsystem() {

      talonRight = new TalonFX(Constants.canID_Right, Constants.CANIVORE_BUS_NAME);
      talonLeft = new TalonFX(Constants.canID_Left, Constants.CANIVORE_BUS_NAME);

        talonRight.getConfigurator()
        .apply(
             new TalonFXConfiguration()
                 .withMotorOutput(
                     new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

    limitSwitch = new DigitalInput(Constants.limitSwitch);

  }

  public void setMotorVoltage(double voltage1){
      talonLeft.setVoltage(voltage1);
      talonRight.setVoltage(voltage1);
  }
  public boolean getLimitSwitchValue(){
      return limitSwitch.get();
  }

}
