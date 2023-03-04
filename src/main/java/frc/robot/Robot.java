// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {

  private Arm arm;
  private Drivetrain drive;
  private Intake intake;

  CommandXboxController xb = new CommandXboxController(Constants.Ports.CONTROLLER);

  @Override
  public void robotInit() {
    this.arm = Arm.getInstance();
    this.drive = Drivetrain.getInstance();
    this.intake = Intake.getInstance();

    xb.rightBumper().onTrue(this.intake.intakeCone()).onFalse(this.intake.holdCone());
    xb.leftBumper().onTrue(this.intake.intakeCube()).onFalse(this.intake.holdCube());
    xb.rightTrigger().onTrue(this.intake.throwItem()).onFalse(this.intake.off());

    xb.y().onTrue(this.arm.setRadians(Constants.Arm.Positions.HIGH_NODE));
    xb.a().onTrue(this.arm.setRadians(Constants.Arm.Positions.FLOOR));
    xb.x().onTrue(this.arm.setRadians(Constants.Arm.Positions.STOW));
    xb.b().onTrue(this.arm.setRadians(Constants.Arm.Positions.MID_NODE));

    drive.setDefaultCommand(Commands.run(() -> {
      drive.curvatureDriveNotCheesyFSFS(-xb.getLeftY(), -xb.getRightX(), xb.getLeftTriggerAxis() > 0.5);
    }, drive));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Time" /* with a capital T */, Timer.getFPGATimestamp());
  }


  @Override
  public void disabledInit() {

  }


  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }


  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {}



  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }


  @Override
  public void testPeriodic() {

  }

  @Override
  public void simulationInit() {

  }


  @Override
  public void simulationPeriodic() {

  }
}
