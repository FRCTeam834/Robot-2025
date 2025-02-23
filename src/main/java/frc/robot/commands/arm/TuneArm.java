// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.arm.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TuneArm extends Command {
  /** Creates a new TuneElevator. */
  private final Arm arm;
  private double voltageSetpoint = 0;
  
  public TuneArm(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    voltageSetpoint = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickInput = OI.xbox.getRightY();
    if (Math.abs(joystickInput) < 0.1) {
      joystickInput = 0;
    }

    voltageSetpoint -= joystickInput * 0.005;

    SmartDashboard.putNumber("Applied Static Voltage", voltageSetpoint);
    arm.setPivotVoltage(MathUtil.clamp(voltageSetpoint, -10, 10));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setPivotVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
