package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
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

  public Drivetrain() {
    frontLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
    backLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
    frontRight.setIdleMode(CANSparkMax.IdleMode.kCoast);
    backRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

    frontRight.setInverted(true);
    backRight.setInverted(true);
  }

  
  private final PigeonIMU gyro = new PigeonIMU(1);

  private final DifferentialDrive diffDrive = new DifferentialDrive(left, right);

  private final static double deadband = 0.1;

  public void curvatureDrive(double speed, double rotation, boolean inplace) {
//    speed = MathUtil.applyDeadband(speed, Drivetrain.deadband);
//    rotation = MathUtil.applyDeadband(rotation, Drivetrain.deadband);
//    if (inplace) {
//      diffDrive.arcadeDrive(speed, rotation);
//      return;
//    }
//    double leftSpeed = speed - Math.abs(speed) * rotation;
//    double rightSpeed = speed + Math.abs(speed) * rotation;
//    double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//    if (maxMagnitude > 1) {
//      leftSpeed /= Math.max(rightSpeed, leftSpeed);
//      rightSpeed /= Math.max(rightSpeed, leftSpeed);
//    }
    diffDrive.curvatureDrive(speed, rotation, inplace);
  }
}
