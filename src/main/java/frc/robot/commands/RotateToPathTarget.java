// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToPathTarget extends Command {
  /** Creates a new RotateToPathTarget. */

  private DriveTrain driveTrain;
  private Rotation2d desiredAngle;

  //private ProfiledPIDController turnController = new ProfiledPIDController(3, 0, 0, new Constraints(Math.PI, 2 * Math.PI));
  private PIDController turnController = new PIDController(3, 0, 0);


  public RotateToPathTarget(DriveTrain driveTrain, Rotation2d desiredAngle) {
    this.driveTrain = driveTrain;
    this.desiredAngle = desiredAngle;

    turnController.enableContinuousInput(0, 2 * Math.PI);
    turnController.setTolerance(Units.degreesToRadians(1));

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //turnController.setGoal(desiredAngle.getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d currentAngle = driveTrain.getYaw();
    //driveTrain.drive(0, 0, turnController.calculate(currentAngle.getRadians()));
    double error = currentAngle.minus(desiredAngle).getRadians();
    driveTrain.drive(0, 0, turnController.calculate(error));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    System.out.println("Done rotating");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return turnController.atGoal();
    return turnController.atSetpoint();
  }
}
