package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TorqueTMP;

// HIM IS 10

public class Arm extends SubsystemBase {
  private static Arm instance;


  private CANSparkMax motor;
  private DutyCycleEncoder aps;
  // TODO: method to set angle (in radians), that should calculate new trapezoid profile

  private double targetPosition;
  private double profilePosition;
  private boolean goalChanged;

  private TorqueTMP tmp;

  public static Arm getInstance() {
    return (Arm.instance == null) ? Arm.instance = new Arm() : Arm.instance;
  }

  public enum ArmState {
    STARTUP,
    POSITION,
    FAULT
  };

  private ArmState currentArmState;
  private double startUpTime;

  private Arm() {
    this.motor = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.motor.restoreFactoryDefaults();
    this.motor.setSmartCurrentLimit(30);
    this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.motor.setInverted(true);
    this.motor.getEncoder().setPositionConversionFactor(Math.PI * 2 / 100);
    this.motor.getEncoder().setVelocityConversionFactor(Math.PI * 2 / 100 / 60.0);

    this.motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 3.85f);
    this.motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    this.motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -1.00f);
    this.motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    this.motor.getPIDController().setOutputRange(-0.5, 0.5);
    this.motor.getPIDController().setP(0.8);

    this.aps = new DutyCycleEncoder(Constants.Ports.ARM_ENCODER_DIO);
//    this.aps.setDistancePerRotation(Math.PI * 2);
//    this.aps.setPositionOffset(0.2839);

    // Minimum: 0.8955
    // 0 Deg: 0.2839
    // Limit: 0.1326

    this.currentArmState = ArmState.STARTUP;
    this.startUpTime = Timer.getFPGATimestamp();

    this.tmp = new TorqueTMP(Math.PI * 1.0, Math.PI * 0.25);

    this.setRadians(this.getMotorRadians());
  }

  private double getAPSRadians() {
    return Math.PI * 2 * (this.aps.getAbsolutePosition() - 0.2839);
  }

  private double getMotorRadians() {
    return this.motor.getEncoder().getPosition();
  }

  private double getMotorRadiansPerSecond() {
    return this.motor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    double time = Timer.getFPGATimestamp();
    ArmState nextArmState = this.currentArmState;
    switch (this.currentArmState) {
      case STARTUP:
        if (time - this.startUpTime < 2.0) {

        } else if (Math.abs(this.getAPSRadians() - this.getMotorRadians()) > Constants.ARM_READY_DEADZONE) {
          this.motor.getEncoder().setPosition(this.getAPSRadians());
        } else {
          nextArmState = ArmState.POSITION;
        }
        break;
      case POSITION:
        if (Math.abs(this.getAPSRadians() - this.getMotorRadians()) > Constants.ARM_READY_DEADZONE) {
          nextArmState = ArmState.FAULT;
        }

        // true == true && true != false
        if (this.goalChanged) {
          this.tmp.generateTrapezoid(this.targetPosition, this.getMotorRadians(), this.getMotorRadiansPerSecond());
          this.goalChanged = false;
        }

        this.tmp.calculateNextSituation(0.020);
        this.profilePosition = this.tmp.getCurrentPosition();
        this.motor.getPIDController().setReference(this.profilePosition, CANSparkMax.ControlType.kPosition);

        break;
      case FAULT:
        break;
    }

    this.currentArmState = nextArmState;

    SmartDashboard.putString("Arm Current State", this.currentArmState.toString());
    SmartDashboard.putNumber("APS (raw)", this.aps.getAbsolutePosition());
    SmartDashboard.putNumber("APS (radians)", this.getAPSRadians());
    SmartDashboard.putNumber("Motor (radians)", this.getMotorRadians());
    SmartDashboard.putNumber("Difference (radians)", this.getAPSRadians() - this.getMotorRadians());
    SmartDashboard.putNumber("Motor output (V)", this.motor.getAppliedOutput() * 12.0);
    SmartDashboard.putNumber("profile pos", this.profilePosition);

  }

  public void testingSet(double val) {
    if (this.currentArmState == ArmState.POSITION) {
      this.motor.set(val);
    } else {
      this.motor.set(0);
    }
  }

  public void setRadians(double rad) {
    this.targetPosition = rad;
    this.goalChanged = true;
  }
}
