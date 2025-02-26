// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.PoseEstimator;

/**
 * This is my rework of AutoDriveWithSpeeds (?)
 */
public class AutoDriveToNearestScoring extends SequentialCommandGroup {
  /** Creates a new AutoDriveToNearestScoring. */
  public AutoDriveToNearestScoring(DriveTrain driveTrain, PoseEstimator poseEstimator) {
    Pose2d closestScoringPose = null;
    Pose2d robotPose = poseEstimator.getPoseEstimate();
  
    double bestCost = Double.MAX_VALUE;
      for(Pose2d scoringPose : Constants.scoringPoses) {
        double distanceError = robotPose.getTranslation().getDistance(scoringPose.getTranslation());
        double angleError = Math.abs(robotPose.getRotation().minus(scoringPose.getRotation()).getRadians());

        double cost = (distanceError * 0.5 + angleError * 5);
        if (cost < bestCost) {
          bestCost = cost;
          closestScoringPose = scoringPose; 
        }
      }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (closestScoringPose == null) return;

    addCommands(
      new AutoDriveToPose(closestScoringPose, driveTrain, poseEstimator)
    );
  }
}
