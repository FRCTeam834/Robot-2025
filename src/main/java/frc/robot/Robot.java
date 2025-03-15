// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elevator.Elevator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    //RobotContainer.limelight.setIMUMode(1);
    RobotContainer.elevator.stop();
    RobotContainer.arm.stop();
    RobotContainer.driveTrain.stop();
  } 

  @Override
  public void disabledPeriodic() {
    if(Constants.VisionConstants.useLL4Gyro) {
      //RobotContainer.limelight.setRobotOrientation(RobotContainer.driveTrain.getYaw());
    }
  }

  @Override
  public void disabledExit() {
    if(Constants.VisionConstants.useLL4Gyro) {
      //RobotContainer.limelight.setIMUMode(2);
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      RobotContainer.estimator.setEstimatorGyroOffset(180);
      System.out.println("Running this thingy");
    } else {
      RobotContainer.estimator.setEstimatorGyroOffset(0);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
