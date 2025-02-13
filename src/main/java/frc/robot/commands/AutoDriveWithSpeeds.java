// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.PoseEstimator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveWithSpeeds extends Command {

  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;

  private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    new PIDController(2, 0, 0), new PIDController(2, 0, 0), 
    new ProfiledPIDController(3, 0, 0,
     new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)))
  );

  private Pose2d desiredPose;
  private Rotation2d desiredHeading;
  private double desiredLinearVelocity;

  public AutoDriveWithSpeeds(Pose2d desiredPose, Rotation2d desiredHeading, double desiredLinearVelocity, DriveTrain driveTrain, PoseEstimator poseEstimator) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;
    this.desiredPose = desiredPose;
    this.desiredHeading = desiredHeading;
    this.desiredLinearVelocity = desiredLinearVelocity;

    holonomicDriveController.setTolerance(new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(2)));

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // field relative blue side
    ChassisSpeeds speeds = holonomicDriveController.calculate(
      poseEstimator.getPoseEstimate(), 
      new Pose2d(desiredPose.getTranslation(), new Rotation2d()), 
      desiredLinearVelocity, // m/s
      desiredHeading
    );

    driveTrain.setDesiredSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("done driving to pose");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return holonomicDriveController.atReference();
  }
}
