package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private final PigeonIMU gyro = new PigeonIMU(1);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.TRACK_WIDTH_METERS);

  private final static double deadband = 0.1;

  private final CANSparkMax frontLeft = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax frontRight = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(backLeft, frontLeft);
  private final MotorControllerGroup right = new MotorControllerGroup(backRight, frontRight);

  private SimpleMotorFeedforward left_feedforward;
  private SimpleMotorFeedforward right_feedforward;

//  private final MotorControllerGroup left = new MotorControllerGroup(
//    new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless), // Front Left
//    new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless)  // Back  Left
//  );
//  private final MotorControllerGroup right = new MotorControllerGroup(
//    new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless), // Front Right
//    new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless)  // Back  Right
//  );

  private Drivetrain() {
    this.left_feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.Feedforward.Left.S, Constants.Drivetrain.Feedforward.Left.V, Constants.Drivetrain.Feedforward.Left.A);
    this.right_feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.Feedforward.Right.S, Constants.Drivetrain.Feedforward.Right.V, Constants.Drivetrain.Feedforward.Right.A);

    frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    backLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    backRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

    frontLeft.setInverted(false);
    backLeft.setInverted(false);
    frontRight.setInverted(true);
    backRight.setInverted(true);

    frontLeft.getEncoder().setPositionConversionFactor(Constants.Drivetrain.POSITION_CONVERSION);
    frontLeft.getEncoder().setVelocityConversionFactor(Constants.Drivetrain.VELOCITY_CONVERSION);
    frontRight.getEncoder().setPositionConversionFactor(Constants.Drivetrain.POSITION_CONVERSION);
    frontRight.getEncoder().setVelocityConversionFactor(Constants.Drivetrain.VELOCITY_CONVERSION);

    if (Constants.Drivetrain.ENABLE_CURRENT_LIMIT) {
      frontLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
      backLeft.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
      frontRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
      backRight.setSmartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
    }
  }

  public static Drivetrain getInstance() {
    return frc.robot.subsystems.Drivetrain.instance == null ? frc.robot.subsystems.Drivetrain.instance = new Drivetrain() : frc.robot.subsystems.Drivetrain.instance;
  }

  private final double kDenominator = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.WHEEL_NONLINEARITY);

  public void curvatureDriveNotCheesyFSFS(double throttle, double wheel, boolean quickTurn) {
    throttle = MathUtil.applyDeadband(throttle, Constants.Drivetrain.DEADBAND);
    wheel = MathUtil.applyDeadband(wheel, Constants.Drivetrain.DEADBAND);

    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.WHEEL_NONLINEARITY * wheel);
      wheel = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.WHEEL_NONLINEARITY * wheel);
      wheel = wheel / (kDenominator * kDenominator) * Math.abs(throttle);
    }

    double vx = throttle * Constants.Drivetrain.DRIVE_MAX_MPS;
    double omega = wheel * Constants.Drivetrain.WHEEL_GAIN;

    DifferentialDriveWheelSpeeds wheels = kinematics.toWheelSpeeds(new ChassisSpeeds(vx, 0.0, omega));


    setOpenLoopWheelSpeed(wheels);
  }

  public void setOpenLoopWheelSpeed(@NotNull DifferentialDriveWheelSpeeds speed) {
//    leftVolts = ;
    left.setVoltage(left_feedforward.calculate(speed.leftMetersPerSecond));
    right.setVoltage(right_feedforward.calculate(speed.rightMetersPerSecond));
  }

}

