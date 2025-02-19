// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TuneElevator extends Command {
  /** Creates a new TuneElevator. */
  private final Elevator elevator;
  private final DoubleSupplier rightJoystickY;

  private double voltageSetpoint = 0;
  
  public TuneElevator(Elevator elevator, DoubleSupplier rightJoystickY) {
    this.elevator = elevator;
    this.rightJoystickY = rightJoystickY;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    voltageSetpoint += MathUtil.clamp(rightJoystickY.getAsDouble() * 0.1, -12, 12);

    SmartDashboard.putNumber("Applied Static Voltage", voltageSetpoint);
    elevator.setElevatorVoltage(voltageSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
