// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.arm.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DumbArm extends Command {
  /** Creates a new DumbArm. */
  private Arm arm;
  private DoubleSupplier input;
  public DumbArm(Arm arm, DoubleSupplier input) {
    this.arm = arm;
    this.input = input;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setPivotVoltage(input.getAsDouble() * 1);
    if (OI.isDPadUpPressed()) {
      arm.setIntakeVoltage(-1.0);
    } else if (OI.isDPadDownPressed()) {
      arm.setIntakeVoltage(1.0);
    } else {
      arm.setIntakeVoltage(0.0);
    }
    // System.out.println("Arm: " + input.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
