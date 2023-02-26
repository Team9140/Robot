// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;

public class Robot extends TimedRobot {

  private Arm arm;


  XboxController xb = new XboxController(0);


  @Override
  public void robotInit() {
    this.arm = Arm.getInstance();


    CommandScheduler.getInstance().registerSubsystem(this.arm);
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
  public void teleopPeriodic() {
//    var result = camera.getLatestResult();
//    if (result.hasTargets()) {
//      PhotonTrackedTarget t = ;
//      System.out.println(result.getBestTarget());
//    }
//    System.out.println(enc.get());

//    arm.testingSet(xb.getLeftY() * 0.5);

    if (xb.getYButtonPressed()) {
      arm.setRadians(Math.PI / 2.0);
    }

    if (xb.getBButtonPressed()) {
      arm.setRadians(Math.PI);
    }
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
