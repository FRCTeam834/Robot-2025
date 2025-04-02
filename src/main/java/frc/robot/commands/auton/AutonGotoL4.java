// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmElevatorGotoPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonGotoL4 extends SequentialCommandGroup {
  /** Creates a new AutonGotoL4. */

  public AutonGotoL4(Elevator elevator, Arm arm) {
    addCommands(new ArmElevatorGotoPosition(ArmConstants.L4_AUTON_ANGLE, ElevatorConstants.L4_AUTON_HEIGHT, arm, elevator, RobotContainer.driveTrain).onlyIf(arm::hasCoral));
  }
}
