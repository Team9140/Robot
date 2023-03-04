package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private final PigeonIMU gyro = new PigeonIMU(1);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DrivetrainConstants.TRACK_WIDTH_METERS);

  private final static double deadband = 0.1;

  private final CANSparkMax frontLeft = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax frontRight = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax backLeft = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(backLeft, frontLeft);
  private final MotorControllerGroup right = new MotorControllerGroup(backRight, frontRight);

//  private final MotorControllerGroup left = new MotorControllerGroup(
//    new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless), // Front Left
//    new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless)  // Back  Left
//  );
//  private final MotorControllerGroup right = new MotorControllerGroup(
//    new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless), // Front Right
//    new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless)  // Back  Right
//  );

  private Drivetrain() {
    frontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
    backLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
    frontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
    backRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

    frontRight.setInverted(true);
    backRight.setInverted(true);
  }

  public static Drivetrain getInstance() {
    return Drivetrain.instance == null ? Drivetrain.instance = new Drivetrain() : Drivetrain.instance;
  }

  private final double kDenominator = Math.sin(Math.PI / 2.0 * Constants.DrivetrainConstants.WHEEL_NONLINEARITY);

  public void curvatureDriveNotCheesyFSFS(double throttle, double wheel, boolean quickTurn) {
    throttle = MathUtil.applyDeadband(throttle, Constants.DrivetrainConstants.DEADBAND);
    wheel = MathUtil.applyDeadband(wheel, Constants.DrivetrainConstants.DEADBAND);

    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * Constants.DrivetrainConstants.WHEEL_NONLINEARITY * wheel);
      wheel = Math.sin(Math.PI / 2.0 * Constants.DrivetrainConstants.WHEEL_NONLINEARITY * wheel);
      wheel = wheel / (kDenominator * kDenominator) * Math.abs(throttle);
    }

    wheel *= Constants.DrivetrainConstants.WHEEL_GAIN;
    DifferentialDriveWheelSpeeds signal = kinematics.toWheelSpeeds(new ChassisSpeeds(throttle, 0.0, wheel));
    signal.desaturate(1);
    this.left.set(signal.leftMetersPerSecond);
    this.right.set(signal.rightMetersPerSecond);
  }
}
