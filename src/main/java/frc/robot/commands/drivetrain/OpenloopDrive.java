// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OpenloopDrive extends Command {
  /** Creates a new OpenloopDrive. */
  private final DriveTrain driveTrain;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final DoubleSupplier omegaSupplier;

  public OpenloopDrive(DriveTrain driveTrain, DoubleSupplier vxSupplier, DoubleSupplier vySupplier, DoubleSupplier omegaSupplier) {
    this.driveTrain = driveTrain;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.omegaSupplier = omegaSupplier;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveOpenLoop(
     vySupplier.getAsDouble() * SwerveConstants.MAX_TRANSLATION_SPEED, 
     -vxSupplier.getAsDouble() * SwerveConstants.MAX_TRANSLATION_SPEED, 
     -omegaSupplier.getAsDouble() * SwerveConstants.MAX_STEER_SPEED
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
