// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.arm.IntakeCoral;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralIntakeSequence extends SequentialCommandGroup {
  /** Creates a new IntakeSequence. */
  public CoralIntakeSequence(Arm arm, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new IntakeCoral(arm),
        new ArmElevatorGotoPosition(ArmConstants.CORAL_INTAKE_ANGLE, ElevatorConstants.INTAKE_HEIGHT, arm, elevator, RobotContainer.driveTrain)
        // arm default command should be ArmDefaultStow
      ).onlyIf(() -> {
        //if (arm.hasCoral()) {
        //  System.out.println("Already have coral");
        //  return false;
        //}
        return true;
      })
    );
  }
}
