package frc.robot;

public final class Constants {
  public static final String CANIVORE_BUS_NAME = "";

  // create constants for the CAN IDs of the left and right shooter motors
  public static final int canID_Right = 0;
  public static final int canID_Left = 0;
    // you can either put placeholder values of 0 or assign a number to these

  // add a constant for the limit switch port
  public static final int limitSwitch = 0;
    // this can either be a placeholder (0) or a value 0 - 9


  // you need constants for the shooting voltages
    // the voltage we give it can be a whole number or a decimal value
    // we need a value for the L4 shooting voltage, the trough shooting voltage, and the intake voltage
      // shooting voltages should be negative, and the L4 value should be larger

  public static final double lFourShootingVoltage = -1.0;
  public static final double troughShootingVoltage = -5.0;
  public static final double intakeVoltage = -4.0;
}
