// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.vision.Limelight;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  private final DriveTrain driveTrain;
  private final Limelight[] cameras; 

  /*
   * [0] = front
   * [1] = back
  */

  private final SwerveDrivePoseEstimator poseEstimator;

  private Field2d combined_field = new Field2d();

  private double estimatorGyroOffset = 0;

  public PoseEstimator(DriveTrain driveTrain, Limelight[] cameras) {
    this.cameras = cameras;
    this.driveTrain = driveTrain;
    poseEstimator = new SwerveDrivePoseEstimator(
      driveTrain.getKinematics(),
      driveTrain.getYaw(),
      driveTrain.getModulePositions(),
      new Pose2d()
    );

    SmartDashboard.putData("combinedfield", combined_field);
    SmartDashboard.putData(this);
  }

  public Pose2d getPoseEstimate() {
    Pose2d estimate = poseEstimator.getEstimatedPosition();
    Pose2d newEstimate = new Pose2d(estimate.getTranslation(), estimate.getRotation().plus(Rotation2d.fromDegrees(estimatorGyroOffset)));
    return newEstimate;
  }

  public Pose2d getPoseEstimateNoOffset() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * 
   * @param pose
   * @param translationTolerance - meters
   * @param rotationTolerance - radians
   * @return
   */
  public boolean isAtPose (Pose2d pose, double translationTolerance, double rotationTolerance) {
    Pose2d diff = getPoseEstimateNoOffset().relativeTo(pose);
    return (
      Math.abs(diff.getX()) < translationTolerance &&
      Math.abs(diff.getY()) < translationTolerance &&
      Math.abs(diff.getRotation().getRadians()) < rotationTolerance
    );
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(driveTrain.getYaw(), driveTrain.getModulePositions(), pose);
  }

  public void resetRotation(Rotation2d rot) {
    poseEstimator.resetRotation(rot);
  }

  /**
   * @param offset - degrees
   */
  public void setEstimatorGyroOffset(double offset) {
    estimatorGyroOffset = offset;
  }

  public double getErrorBetweenCams(LimelightHelpers.PoseEstimate firstPose, LimelightHelpers.PoseEstimate secondPose) {
    if (firstPose == null || secondPose == null) return -1;
    double dist = firstPose.pose.getTranslation().getDistance(secondPose.pose.getTranslation());
    return dist;
  }

  @Override
  public void periodic() {
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), driveTrain.getYaw(), driveTrain.getModulePositions());

    cameras[0].setRobotOrientation(getPoseEstimateNoOffset().getRotation()); 
    cameras[1].setRobotOrientation(getPoseEstimateNoOffset().getRotation());
    LimelightHelpers.PoseEstimate[] cam_estimates = {cameras[0].getPoseEstimate2d(), cameras[1].getPoseEstimate2d()};

    if(!VisionConstants.useVisionPoseEstimator) return;
    
    if(Constants.VisionConstants.STRATEGY == Constants.LimelightStrategies.ALL_ESTIMATES) {
      for(LimelightHelpers.PoseEstimate estimate : cam_estimates) {
        if(estimate != null) poseEstimator.addVisionMeasurement(
          estimate.pose, 
          estimate.timestampSeconds,
          VecBuilder.fill(
            Math.pow(0.5, estimate.tagCount) * 2 * estimate.avgTagDist,
            Math.pow(0.5, estimate.tagCount) * 2 * estimate.avgTagDist,
            Double.POSITIVE_INFINITY
        ));
      }
    }

    if(Constants.VisionConstants.STRATEGY == Constants.LimelightStrategies.AVERAGE_ESTIMATE) {
      double averagex = (cam_estimates[0].pose.getX() + cam_estimates[0].pose.getX()) / cam_estimates.length;
      double averagey = (cam_estimates[1].pose.getY() + cam_estimates[1].pose.getY()) / cam_estimates.length;
      
      double theta_y_sum = (
        Math.sin(cam_estimates[0].pose.getRotation().getRadians()) + 
        Math.sin(cam_estimates[1].pose.getRotation().getRadians())
      );

      double theta_x_sum = (
        Math.cos(cam_estimates[0].pose.getRotation().getRadians()) + 
        Math.cos(cam_estimates[1].pose.getRotation().getRadians())
      );

      double theta_avg = Math.atan2(theta_y_sum / cam_estimates.length, theta_x_sum / cam_estimates.length);

      Pose2d average_pose = new Pose2d(averagex, averagey, new Rotation2d(theta_avg));

      poseEstimator.addVisionMeasurement(average_pose, Math.max(cam_estimates[0].timestampSeconds, cam_estimates[1].timestampSeconds));
    }

    if(Constants.VisionConstants.STRATEGY == Constants.LimelightStrategies.BEST_ESTIMATE) {
      double bestDistance = Double.MAX_VALUE;
      LimelightHelpers.PoseEstimate bestEstimate = cam_estimates[0];

      for(LimelightHelpers.PoseEstimate estimate : cam_estimates) {
        for(RawFiducial fiducial : estimate.rawFiducials) {
          if (fiducial.distToCamera < bestDistance) {
            bestDistance = fiducial.distToCamera;
            bestEstimate = estimate;
          }
        }
      }

      poseEstimator.addVisionMeasurement(bestEstimate.pose, bestEstimate.timestampSeconds);
    }

    combined_field.setRobotPose(getPoseEstimate());
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("PoseEstimator");

    if(!Constants.tuningMode) return;

    builder.addDoubleProperty("poseEstimator Yaw", () -> { return getPoseEstimate().getRotation().getDegrees(); }, null);
    builder.addDoubleProperty("error between cams", () -> { return getErrorBetweenCams(cameras[0].getPoseEstimate2d(), cameras[1].getPoseEstimate2d()); }, null);
  }
}
