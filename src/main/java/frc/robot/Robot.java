// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Robot extends TimedRobot {

  private Arm arm;
  private Drivetrain drive;
  private Intake intake;

  private SequentialCommandGroup autoSequence;

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
      double turnMultiplier = (Math.sin(arm.getTargetStatePosition()) >= 0)
          ? Constants.Drivetrain.ARM_EXTENDED_TURN_MULTIPLIER
          : Constants.Drivetrain.ARM_STOW_TURN_MULTIPLIER;
      drive.curvatureDrive(-xb.getLeftY(), -xb.getRightX() * turnMultiplier,
          xb.getLeftTriggerAxis() > 0.5);
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
    CommandBase midCubeDriveAuto = new SequentialCommandGroup(
        this.intake.holdCube().raceWith(new WaitCommand(0.1)),
        this.arm.setRadians(Constants.Arm.Positions.MID_NODE),
        new WaitCommand(2.0),
        this.intake.throwItem().raceWith(new WaitCommand(1.0)),
        this.arm.setRadians(Constants.Arm.Positions.STOW),
        new WaitCommand(2.0),
        this.intake.off().raceWith(new WaitCommand(0.1)),
        this.drive.crawlDistance(Constants.Auto.DRIVE_BACK_METERS, -Constants.Auto.DRIVE_MPS));

    CommandBase highCubeDriveAuto = new SequentialCommandGroup(
        this.drive.resetEncoders(),
        this.drive.crawlDistance(0.4, Constants.Auto.DRIVE_MPS / 4.0).raceWith(new WaitCommand(2.5)),
        this.intake.holdCube().raceWith(new WaitCommand(0.05)),
        this.arm.setRadians(Constants.Arm.Positions.HIGH_NODE),
        new WaitCommand(2.0),
        this.intake.throwItem().raceWith(new WaitCommand(1.0)),
        this.arm.setRadians(Constants.Arm.Positions.STOW),
        new WaitCommand(2.0),
        this.intake.off().raceWith(new WaitCommand(0.05)),
        this.drive.resetEncoders(),
        this.drive.crawlDistance(Constants.Auto.DRIVE_BACK_METERS, -Constants.Auto.DRIVE_MPS));

    CommandBase balanceAuto = new SequentialCommandGroup(
        this.drive.crawlUntilTilt(Constants.Auto.DRIVE_MPS),
        this.drive.crawlUntilLevel(Constants.Auto.LEVEL_MPS));

    CommandBase turnInPlaceTest = new SequentialCommandGroup(
        this.drive.turnInPlace(90),
        this.drive.turnInPlace(-90),
        this.drive.turnInPlace(180));

    // CommandScheduler.getInstance().cancelAll();

    highCubeDriveAuto.schedule();
    // balanceAuto.schedule();
    // turnInPlaceTest.schedule();
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
  }

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
