package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// HIM IS ELEVEN MAKE HIM DO THE THING

public class Intake extends SubsystemBase {
  /*

  CAN spark max with brushed motor

   */
  private static Intake instance;
  private CANSparkMax motor;

  public enum IntakeState {
    INTAKE_CUBE,
    INTAKE_CONE,
    HOLD,
    THROW,
    OFF
  }

  private IntakeState state;
  private IntakeState lastIntakeState;

  private Intake() {
    this.motor = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushed);
    this.state = IntakeState.OFF;
    this.lastIntakeState = IntakeState.OFF;
  }

  public static Intake getInstance() {
    return Intake.instance == null ? Intake.instance = new Intake() : Intake.instance;
  }

  public void setState(IntakeState newState) {
    this.state = newState;
  }

  @Override
  public void periodic() {
    switch (this.state) {
      case INTAKE_CUBE:
        motor.set(-1.0);
        this.lastIntakeState = IntakeState.INTAKE_CUBE;
        break;
      case INTAKE_CONE:
        this.lastIntakeState = IntakeState.INTAKE_CONE;
        motor.set(1.0);
        break;
      case HOLD:
        if (this.lastIntakeState == IntakeState.INTAKE_CUBE) {
        motor.set(-0.2);
        } else if (this.lastIntakeState == IntakeState.INTAKE_CONE) {
          motor.set(0.2);
        } else {
          this.state = IntakeState.OFF;
        }
        break;
      case THROW:
        if (this.lastIntakeState == IntakeState.INTAKE_CUBE){
        motor.set(1.0);
        } else if (this.lastIntakeState == IntakeState.INTAKE_CONE) {
          motor.set(-1.0);
        } else {
          this.state = IntakeState.OFF;
        }
        break;
      case OFF:
        lastIntakeState = IntakeState.OFF;
        motor.set(0);
        break;
    }
  }

  /**
   * intake cube
   * intake cone
   * hold
   * throw
   *
   * (throw/hold/intake) (cube/cone), off
   */
}
