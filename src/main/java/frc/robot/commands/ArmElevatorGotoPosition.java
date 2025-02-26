// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmElevatorSuperconstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmElevatorGotoPosition extends SequentialCommandGroup {
  private Arm arm;
  private Elevator elevator;
  private double desiredArmAngle;
  private double desiredElevatorHeight;
  /** Creates a new ArmElevatorGotoPosition. */
  public ArmElevatorGotoPosition(double desiredArmAngle, double desiredElevatorHeight, Arm arm, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arm = arm;
    this.elevator = elevator;
    this.desiredArmAngle = desiredArmAngle;
    this.desiredElevatorHeight = desiredElevatorHeight;

    addCommands(
      getSequence().onlyIf(() -> {
        if (
          desiredArmAngle > ArmElevatorSuperconstants.NOCOLLISION_MIN_ARM_ANGLE &&
          desiredElevatorHeight > ArmElevatorSuperconstants.NOCOLLISION_MAX_ELEVATOR_HEIGHT
        ) {
          // this state is not attainable
          System.out.println("Unattainable arm and elevator angle set");
          return false;
        }
        return true;
      })
    );
  }

  /**
   * This is the "dumb" method
   * If collision might occur, it will run either arm / elevator first until out of the way
   * This will be fine if the arm and elevator move quickly
   * I also decided this should be good enough because if our arm / elevator gets messed up somehow,
   * this will still work. If we instead had a fine-tuned timer system, then if our system changes
   * we will destroy our arm and elevator.
   * @return
   */
  private Command getSequence () {
    // IMPORTANT NOTE: Remember that negative angle is farther out, therefore it's >
    boolean currentArmCollides = arm.getCurrentPivotAngle() > ArmElevatorSuperconstants.NOCOLLISION_MIN_ARM_ANGLE;
    boolean currentElevatorCollides = elevator.getElevatorHeight() > ArmElevatorSuperconstants.NOCOLLISION_MAX_ELEVATOR_HEIGHT;

    boolean desiredArmCollides = desiredArmAngle > ArmElevatorSuperconstants.NOCOLLISION_MIN_ARM_ANGLE;
    boolean desiredElevatorCollides = desiredElevatorHeight > ArmElevatorSuperconstants.NOCOLLISION_MAX_ELEVATOR_HEIGHT;

    if (
      (!currentArmCollides && !desiredArmCollides) || // arm always stays out of the way
      (!currentElevatorCollides && !desiredElevatorCollides) // elevator always stays out of the way
    ) {
      // will never collide
      System.out.println("CASE: Never Collide: " + desiredArmAngle + ", " + desiredElevatorHeight);
      return new InstantCommand(() -> {
        arm.setDesiredPivotAngle(desiredArmAngle);
        elevator.setDesiredHeight(desiredElevatorHeight);
      });
    } else if (
      (!currentElevatorCollides && desiredElevatorCollides && currentArmCollides && !desiredArmCollides) // (low, in) -> (high, out)
    ) {
      // move arm first
      System.out.println("CASE: Move arm first: " + desiredArmAngle + ", " + desiredElevatorHeight);
      return new SequentialCommandGroup(
        new InstantCommand(() -> {
          elevator.setDesiredHeight(elevator.getElevatorHeight()); // keep elevator still
          arm.setDesiredPivotAngle(desiredArmAngle);
        }),
        new WaitUntilCommand(() -> {
          return arm.getCurrentPivotAngle() < ArmElevatorSuperconstants.NOCOLLISION_MIN_ARM_ANGLE;
        }),
        new InstantCommand(() -> {
          elevator.setDesiredHeight(desiredElevatorHeight);
        })
      );
    } else if (
      (currentElevatorCollides && !desiredElevatorCollides && !currentArmCollides && desiredArmCollides) // (high, out) -> (low, in)
    ) {
      // move elevator first
      System.out.println("CASE: Move elevator first: " + desiredArmAngle + ", " + desiredElevatorHeight);
      return new SequentialCommandGroup(
        new InstantCommand(() -> {
          arm.setDesiredPivotAngle(arm.getCurrentPivotAngle()); // keep arm still
          elevator.setDesiredHeight(desiredElevatorHeight);
        }),
        new WaitUntilCommand(() -> {
          return elevator.getElevatorHeight() < ArmElevatorSuperconstants.NOCOLLISION_MAX_ELEVATOR_HEIGHT;
        }),
        new InstantCommand(() -> {
          arm.setDesiredPivotAngle(desiredArmAngle);
        })
      );
    }

    return new InstantCommand(() -> {
      System.out.println("Some case was not handled... " + desiredArmAngle + ", " + desiredElevatorHeight);
    });
    
    // this should cover all cases?
    // note: (high, in) is physically impossible
    // -> (high, in) is handled in the onlyIf block
  }
}
