package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// HIM IS ELEVEN MAKE HIM DO THE THING

public class Intake extends SubsystemBase {
  /*

  CAN spark max with brushed motor

   */

  private CANSparkMax doodad = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
}
