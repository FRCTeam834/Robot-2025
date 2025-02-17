// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.OI;
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

  private double desiredLinearVelocity;

  private double joystickTolerance = 0.15;
  private boolean doesCalculateScoringPosition = true;

  private Pose2d closestScoringPose;

  public AutoDriveWithSpeeds(DriveTrain driveTrain, PoseEstimator poseEstimator) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;

    holonomicDriveController.setTolerance(new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(2)));

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightJoystickx = OI.getRightJoystickX();
    double rightJoysticky = OI.getRightJoystickY();
    double leftJoystickx = OI.getLeftJoystickX();

    Pose2d robotPose = poseEstimator.getPoseEstimate();

    // While matt is manually driving robot recalculate most optimal scoring position
    if (doesCalculateScoringPosition) {
      double bestCost = Double.MAX_VALUE;

      for(Constants.SCORING_POSES_BLUE scoringPose : Constants.SCORING_POSES_BLUE.values()) {
        double distanceError = robotPose.getTranslation().getDistance(scoringPose.getPose().getTranslation());
        double angleError = Math.abs(robotPose.getRotation().minus(scoringPose.getPose().getRotation()).getDegrees());

        double cost = (distanceError * 0.5 + angleError * 5); // angleError more important than distanceError
        if (cost < bestCost) {
          bestCost = cost;
          closestScoringPose = scoringPose.getPose(); 
        }
      }
    }

    if(Math.abs(rightJoystickx) > joystickTolerance || Math.abs(rightJoysticky) > joystickTolerance || Math.abs(leftJoystickx) > joystickTolerance) {
      driveTrain.drive(
        rightJoystickx * SwerveConstants.MAX_TRANSLATION_SPEED, 
        -rightJoysticky * SwerveConstants.MAX_TRANSLATION_SPEED, 
        -leftJoystickx * SwerveConstants.MAX_STEER_SPEED
      );
      doesCalculateScoringPosition = true;
    } else {
      ChassisSpeeds speeds = holonomicDriveController.calculate(
        robotPose, 
        new Pose2d(closestScoringPose.getTranslation(), new Rotation2d()), 
        desiredLinearVelocity, // m/s
        closestScoringPose.getRotation()
      );

      driveTrain.setDesiredSpeedsFromHolonomicController(speeds);
      doesCalculateScoringPosition = false;
    }
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
