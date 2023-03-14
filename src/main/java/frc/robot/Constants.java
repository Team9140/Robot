// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Drivetrain {
    public static final double DEADBAND = 0.15;
    public static final double WHEEL_TURN_GAIN = 12.0;
    public static final double WHEEL_NONLINEARITY = 0.07;
    public static final double TRACK_WIDTH_INCHES = 17.5;
    public static final double TRACK_WIDTH_METERS = TRACK_WIDTH_INCHES * Units.METERS_PER_INCH;

    public static final double DRIVE_RATIO = 8.45;

    public static final double MOTOR_RPM = 5820.0;

    public static final double WHEEL_DIAMETER = 6.0;

    public static final double DRIVE_MAX_MPS = (MOTOR_RPM / DRIVE_RATIO / Units.SECONDS_PER_MINUTE) * WHEEL_DIAMETER
        * Math.PI * Units.METERS_PER_INCH;
    public static final double POSITION_CONVERSION = (1 / DRIVE_RATIO) * WHEEL_DIAMETER * Math.PI
        * Units.METERS_PER_INCH;
    public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / Units.SECONDS_PER_MINUTE;

    // Set the current limit, an integer in amps, for the drivetrain.
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final int CURRENT_LIMIT = 40;

    public static final double ARM_EXTENDED_TURN_MULTIPLIER = 0.5;
    public static final double ARM_STOW_TURN_MULTIPLIER = 1.0;
    public static final double CHARGE_STATION_PITCH = 13;
    public static final double LEVEL_PITCH = 6.0;
    public static final double LIMITER = 0.5;

    public static class Feedforward {
      public static class Left {
        public static final double S = 0.11892;
        public static final double V = 2.2116;
        public static final double A = 0.11901;
      }

      public static class Right {
        public static final double S = 0.11869;
        public static final double V = 2.2123;
        public static final double A = 0.14048;
      }
    }
  }

  public static class Arm {
    public static class Positions {
      public static final double STOW = 3.854;
      public static final double HIGH_NODE = 0.616;
      public static final double MID_NODE = 0.299604;
      public static final double FLOOR = -1.0;
    }

    public static final double P = 3.6935;
    public static final double I = 0.0;
    public static final double D = 0.61277;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double READY_DEADZONE = 0.025;
    public static final double FAULT_DEADZONE = 0.25;

    public static final double POSITION_CONVERSION = Constants.Units.RADIANS_PER_ROTATION / 100.0;

    public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / Units.SECONDS_PER_MINUTE;

    public static final int CURRENT_LIMIT = 40;
  }

  public static class IntakeConstants {
    public static final int INTAKE_CURRENT_LIMIT_AMPS = 35;
    public static final int HOLD_CURRENT_LIMIT_AMPS = 8;
    public static final double INTAKE_CONE_VOLTS = 12.0;
    public static final double INTAKE_CUBE_VOLTS = -8.0;
    public static final double HOLD_CONE_VOLTS = 3.0;
    public static final double HOLD_CUBE_VOLTS = -3.0;
    public static final double THROW_CONE_VOLTS = -8.0;
    public static final double THROW_CUBE_VOLTS = 8.0;
    public static final double OFF = 0.0;
  }

  public static class Ports {
    public static final int CONTROLLER = 0;
    public static final int ARM_ENCODER_DIO = 9;
  }

  public static class Units {
    public static final double SECONDS_PER_MINUTE = 60.0;

    public static final double METERS_PER_INCH = 0.0254;

    public static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;
  }

  public static class Auto {

    public static final double DRIVE_BACK_METERS = -5.0;
    public static final double DRIVE_MPS = 1.0;
    public static final double LEVEL_MPS = 0.5;

  }
}
