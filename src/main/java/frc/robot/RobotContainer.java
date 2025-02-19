// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.MoveWithXbox;
import frc.robot.commands.testPID;
import frc.robot.commands.tuneElevator;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  public static Joystick leftJoystick = new Joystick(1);
  public static Elevator elevator = new Elevator();

  public static JoystickButton leftJoystick6 = new JoystickButton(leftJoystick, 6);
  public static JoystickButton leftJoystick7 = new JoystickButton(leftJoystick, 7);
  public static JoystickButton leftJoystick10 = new JoystickButton(leftJoystick, 10);
  public static JoystickButton leftJoystick11 = new JoystickButton(leftJoystick, 11);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    elevator.setDefaultCommand(new tuneElevator(elevator, OI::getXboxRightJoystickY));

    leftJoystick6.whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    leftJoystick7.whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    leftJoystick10.whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //leftJoystick11.whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    leftJoystick11.whileTrue(new testPID(elevator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
