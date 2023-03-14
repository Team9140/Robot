package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private CANSparkMax motor;

  private double nextThrowVolts = Constants.IntakeConstants.OFF;

  private Intake() {
    this.motor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushed);
  }

  public static Intake getInstance() {
    return Intake.instance == null ? Intake.instance = new Intake() : Intake.instance;
  }

  public CommandBase intakeCone() {
    return run(() -> {
      this.motor.setSmartCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
      this.motor.set(Constants.IntakeConstants.INTAKE_CONE_VOLTS);
    });
  }

  public CommandBase holdCone() {
    return run(() -> {
      this.motor.setSmartCurrentLimit(Constants.IntakeConstants.HOLD_CURRENT_LIMIT_AMPS);
      this.motor.setVoltage(Constants.IntakeConstants.HOLD_CONE_VOLTS);
      this.nextThrowVolts = Constants.IntakeConstants.THROW_CONE_VOLTS;
    });
  }

  public CommandBase intakeCube() {
    return run(() -> {
      this.motor.set(Constants.IntakeConstants.INTAKE_CUBE_VOLTS);
      this.motor.setSmartCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
    });
  }

  public CommandBase holdCube() {
    return run(() -> {
      this.motor.setSmartCurrentLimit(Constants.IntakeConstants.HOLD_CURRENT_LIMIT_AMPS);
      this.motor.setVoltage(Constants.IntakeConstants.HOLD_CUBE_VOLTS);
      this.nextThrowVolts = Constants.IntakeConstants.THROW_CUBE_VOLTS;
    });
  }

  public CommandBase throwItem() {
    return run(() -> {
      this.motor.setVoltage(this.nextThrowVolts);
      this.motor.setSmartCurrentLimit(Constants.IntakeConstants.INTAKE_CURRENT_LIMIT_AMPS);
    });
  }

  public Command off() {
    return run(() -> this.motor.setVoltage(Constants.IntakeConstants.OFF));
  }

  @Override
  public void periodic() {}
}
