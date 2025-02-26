// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmElevatorGotoPosition;
import frc.robot.commands.arm.IntakeCoral;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonIntake extends ParallelCommandGroup {
  /** Creates a new AutonIntake. */
  public AutonIntake(Arm arm, Elevator elevator) {
    /**
     * NOTE: I'm not sure if pathplanenr waits for commands to finish before executing the next path
     * If so, change parallel group(AutonIntake, followPath) to deadline in the pathplanenr GUI
     */
    addCommands(
      new ArmElevatorGotoPosition(ArmConstants.CORAL_INTAKE_ANGLE, ElevatorConstants.INTAKE_HEIGHT, arm, elevator),
      new IntakeCoral(arm)
    );
  }
}
