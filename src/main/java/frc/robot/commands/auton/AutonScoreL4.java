// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArmElevatorGotoPosition;
import frc.robot.commands.arm.OuttakeCoral;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utility.PoseEstimator;
import frc.robot.commands.auton.AutonOuttakeCoral;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonScoreL4 extends SequentialCommandGroup {
  /** Creates a new AutonScoreL4. */
  public AutonScoreL4(DriveTrain driveTrain, PoseEstimator poseEstimator, Arm arm, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        //new ArmElevatorGotoPosition(ArmConstants.L4_ANGLE, ElevatorConstants.L4_HEIGHT, arm, elevator),
        new AutonOuttakeCoral(arm, elevator)
      ).onlyIf(arm::hasCoral)
    );
  }
}
