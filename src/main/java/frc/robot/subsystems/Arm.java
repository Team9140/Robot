package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

// HIM IS 10

public class Arm extends SubsystemBase {
  // TODO: make instance of trapezoid profile here
//  private final TrapezoidProfileCommand armCommand = ;

  // TODO: make instance of CAN spark max with brushless motor

  private final CANSparkMax arm = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
  // TODO: make instance of duty cycle encoder
  // TODO: method to set angle (in radians), that should calculate new trapezoid profile
}
