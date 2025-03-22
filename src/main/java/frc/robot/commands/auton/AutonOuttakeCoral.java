// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.util.cleanup.CleanupPool;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonOuttakeCoral extends Command {
  /** Creates a new RunIntake. */
  private Arm arm;
  private Elevator elevator;

  private Timer timer = new Timer();
  private Timer cupTimer = new Timer();

  private boolean finished = false;
  private boolean doCupping = false;

  public AutonOuttakeCoral(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    doCupping = false;

    cupTimer.stop();
    cupTimer.reset();

    if(elevator.getElevatorHeight() > ElevatorConstants.L3_HEIGHT + 0.5) {
      arm.setDesiredPivotAngle(ArmConstants.L4_CUP_ANGLE);
      arm.setIntakeVoltage(0);
      cupTimer.start();
      doCupping = true;
    } else {
      arm.setIntakeVoltage(8);
    }


    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(doCupping && cupTimer.get() > 0.3) {
      arm.setIntakeVoltage(8);
      timer.start();
    }

    if (!arm.hasCoral() && timer.get() == 0 && !doCupping) {
      timer.start();
    }

    if (timer.get() > 0.75 && doCupping) {
      arm.setDesiredPivotAngle(ArmConstants.L4_ANGLE);
      finished = true;
    }

    if(timer.get() > 2) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setIntakeVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
