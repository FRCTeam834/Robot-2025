// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.arm.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
  /** Creates a new RunIntake. */
  private Arm arm;
  private LinearFilter filter = LinearFilter.movingAverage(5);

  public IntakeAlgae(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setIntakeVoltage(-12);
    filter.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current = filter.calculate(arm.getIntakeOutputCurrent());

    if (current > 6.0) { // some value
      arm.currentPiece = GamePiece.ALGAE;
      cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setIntakeVoltage(0.0); // could have some small holding voltage
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
