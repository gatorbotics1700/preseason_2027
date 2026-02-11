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

package frc.robot.subsystems.mech.MechIOs;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real hood IO using TalonFX. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor =
      new TalonFX(Constants.HOOD_MOTOR_CAN_ID, TunerConstants.mechCANBus);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private TalonFXConfiguration talonFXConfigs;
  private static MotionMagicExpoVoltage m_request;

  private static final double HOOD_SHAFT_REVS_PER_MECH_REV = 155 / 15.0;
  private static final double HOOD_GEARBOX_RATIO = 9.0;

  public HoodIOTalonFX() {
    // MOTION MAGIC PID/FEEDFORWARD CONFIGS // TODO: must tune everything!!
    talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.withMotorOutput(
        new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    // TODO: make tuneable constants
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    // Add 0.2128 V output to overcome gravity (tuned in early feedforward
    // testing)
    slot0Configs.kG = 0.2128;

    // Add 0.01 V output to overcome static friction (just a guesstimate, but this might just be 0
    slot0Configs.kS = 0.25;

    slot0Configs.kV = 0.16; // A velocity target of 1 rps results in 0.12 V output

    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output

    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12V output

    slot0Configs.kI = 0; // no output for integrated error

    slot0Configs.kD = 0.1; // a velocity error of 1 rps results in 0.1 V output

    // MOTION MAGIC EXPO
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    motionMagicConfigs.MotionMagicCruiseVelocity = 0; // unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.16; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    hoodMotor.getConfigurator().apply(talonFXConfigs);

    m_request = new MotionMagicExpoVoltage(0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.positionRevs = hoodMotor.getPosition().getValueAsDouble();
    inputs.velocityRevsPerSec = hoodMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setSpeed(Rotation2d desiredAngle) {
    hoodMotor.setControl(m_request.withPosition(degreesToRevs(desiredAngle.getDegrees())));
  }

  public double degreesToRevs(double hoodAngleDegrees) {
    return hoodAngleDegrees / 360.0 * HOOD_SHAFT_REVS_PER_MECH_REV * HOOD_GEARBOX_RATIO;
  }

  public Rotation2d getCurrentAngle() {
    double motorPositionRevs = hoodMotor.getPosition().getValueAsDouble();
    double hoodAngleDegrees =
        motorPositionRevs / HOOD_GEARBOX_RATIO / HOOD_SHAFT_REVS_PER_MECH_REV * 360 % 360;
    return new Rotation2d(
        Math.toRadians(
            hoodAngleDegrees)); // TODO: figure out how to use the fromDegrees method because it
    // seems nicer :/
  }

  public void setHoodVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }
}
