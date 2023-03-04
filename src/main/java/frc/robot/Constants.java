// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DrivetrainConstants {
    public static final double DEADBAND = 0.02;
    public static final double WHEEL_GAIN = 0.05;
    public static final double WHEEL_NONLINEARITY = 0.05;
    public static final double TRACK_WIDTH_INCHES = 17.5;
    public static final double TRACK_WIDTH_METERS = TRACK_WIDTH_INCHES * 0.0254;
  }

  public static class ArmPositions {
    public static final double HIGH_NODE = Math.PI;
    public static final double FLOOR = 0.0;
    public static final double STOW = 0;
    public static final double MID_NODE = 0;
  }

  public static class IntakeConstants {
    public static final int INTAKE_CURRENT_LIMIT_AMPS = 25;
    public static final int HOLD_CURRENT_LIMIT_AMPS = 5;
    public static final double INTAKE_CONE_VOLTS = 12.0;
    public static final double INTAKE_CUBE_VOLTS = -12.0;
    public static final double HOLD_CONE_VOLTS = 2.0;
    public static final double HOLD_CUBE_VOLTS = -2.0;
    public static final double THROW_CONE_VOLTS = -12.0;
    public static final double THROW_CUBE_VOLTS = 12.0;
    public static final double OFF = 0.0;
  }

  public static final double ARM_READY_DEADZONE = 0.25;
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class Ports {
    public static final int ARM_ENCODER_DIO = 9;
  }
}
