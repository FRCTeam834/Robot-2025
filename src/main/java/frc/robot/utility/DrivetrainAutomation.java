// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DriveTrain;

public class DrivetrainAutomation extends SubsystemBase {
  private final PoseEstimator estimator;
  private final DriveTrain driveTrain;

  private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    new PIDController(2, 0, 0), new PIDController(2, 0, 0), 
    new ProfiledPIDController(3, 0, 0,
      new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)))
  );

  public DrivetrainAutomation(DriveTrain driveTrain, PoseEstimator estimator) {
    this.estimator = estimator;
    this.driveTrain = driveTrain;

    holonomicDriveController.setTolerance(new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(2)));
  }

  public void driveToPose(Pose2d desiredPose, double desiredLinearVelocity, Rotation2d desiredHeading) {
    ChassisSpeeds speeds = holonomicDriveController.calculate(
      estimator.getPoseEstimate(), 
      new Pose2d(desiredPose.getTranslation(), new Rotation2d()), 
      desiredLinearVelocity, // m/s
      desiredHeading
    );

    driveTrain.setDesiredSpeeds(speeds);
  }

  public boolean atSetpoint() {
    return holonomicDriveController.atReference();
  }

  public Pose2d getClosestScoringAprilTagPose() {
    double bestDistance = Double.MIN_VALUE;
    Pose2d bestPose = null;

    Pose2d robotPose = estimator.getPoseEstimate();

    for(Constants.BLUE_TAG_POSES pose : Constants.BLUE_TAG_POSES.values()) {
      Pose2d targetPose = pose.getPose();

      double distance = Math.hypot(robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY());

      if (distance < bestDistance) {
        bestDistance = distance;
        bestPose = pose.getPose();
      }
    }

    return bestPose;
  }

  public Pose2d getClosestScoringPose() {
    Pose2d closestTagPose = getClosestScoringAprilTagPose();

    double newX1 = closestTagPose.getX() + (6 * Math.sin(closestTagPose.getRotation().getRadians()));
    double newY1 = closestTagPose.getY() + (6 * Math.cos(closestTagPose.getRotation().getRadians()));
    Pose2d pose1 = new Pose2d(newX1, newY1, closestTagPose.getRotation());

    double newX2 = closestTagPose.getX() - (6 * Math.sin(closestTagPose.getRotation().getRadians()));
    double newY2 = closestTagPose.getY() - (6 * Math.cos(closestTagPose.getRotation().getRadians()));
    Pose2d pose2 = new Pose2d(newX2, newY2, closestTagPose.getRotation());

    double dist1 = estimator.getPoseEstimate().getTranslation().getDistance(pose1.getTranslation());
    double dist2 = estimator.getPoseEstimate().getTranslation().getDistance(pose2.getTranslation());

    if(dist1 < dist2) {
      return pose1;
    }
    return pose2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
