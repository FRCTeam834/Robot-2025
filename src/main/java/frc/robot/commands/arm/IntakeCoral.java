// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.arm.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  /** Creates a new RunIntake. */
  private Arm arm;
  private Timer timer = new Timer();
  private boolean ended = false;

  private double lastAngle;
  private boolean hasCoral;

  public IntakeCoral(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ended = false;
    hasCoral = false;
    arm.setIntakeVoltage(8); // 4 was good
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.hasCoral() && !hasCoral) {
      arm.setIntakeVoltage(1); // a lower voltage to move it past elevator stage
      lastAngle = arm.getIntakeAngle();
      hasCoral = true;
    }

    if (hasCoral) System.out.println(arm.getIntakeAngle() - lastAngle);

    if (hasCoral && Math.abs(arm.getIntakeAngle() - lastAngle) > 50) {
      ended = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setIntakeVoltage(0.0);

    if (arm.hasCoral()) {
      arm.currentPiece = GamePiece.CORAL;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ended;
  }
}
