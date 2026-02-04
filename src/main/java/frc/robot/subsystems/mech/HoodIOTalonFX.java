// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.mech;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real hood IO using TalonFX. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  public HoodIOTalonFX() {
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));
    motor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionRevs = motor.getPosition().getValue();
    inputs.velocityRevsPerSec = motor.getVelocity().getValue();
  }

  @Override
  public void setSpeed(double speed) {
    motor.setControl(dutyCycleOut.withOutput(speed));
  }
}
