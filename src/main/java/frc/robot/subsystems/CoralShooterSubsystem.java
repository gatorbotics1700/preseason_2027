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
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DigitalInput limitSwitch;

  /** Creates a new CoralShooterSubsystem. */
  public CoralShooterSubsystem() {
    leftMotor = new TalonFX(Constants.LEFT_SHOOTER_MOTOR_ID, Constants.CANIVORE_BUS_NAME);
    rightMotor = new TalonFX(Constants.RIGHT_SHOOTER_MOTOR_ID, Constants.CANIVORE_BUS_NAME);

    rightMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

    limitSwitch = new DigitalInput(Constants.LIMIT_SWITCH_PORT);
  }

  public void setMotorVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }
}
