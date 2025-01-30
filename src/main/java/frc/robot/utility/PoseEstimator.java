// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.vision.Limelight;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  private final DriveTrain driveTrain;
  private final Limelight limelight;
  private final SwerveDrivePoseEstimator poseEstimator;

  private LimelightHelpers.PoseEstimate estimate;

  private Field2d field = new Field2d();

  public PoseEstimator(DriveTrain driveTrain, Limelight limelight) {
    this.limelight = limelight;
    this.driveTrain = driveTrain;
    poseEstimator = new SwerveDrivePoseEstimator(
      driveTrain.getKinematics(),
      driveTrain.getYaw(),
      driveTrain.getModulePositions(),
      new Pose2d()
    );
    SmartDashboard.putData("KellersField", field);
    SmartDashboard.putData(this);
  }

  public Pose2d getPoseEstimate() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(driveTrain.getYaw(), driveTrain.getModulePositions(), pose);
  }

  @Override
  public void periodic() {
    estimate = limelight.getPoseEstimate2d();
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), driveTrain.getYaw(), driveTrain.getModulePositions());
    
    field.setRobotPose(estimate.pose);

    if(!VisionConstants.useVisionPoseEstimator) return;
    if(VisionConstants.useMegatag2) limelight.setRobotOrientation(driveTrain.getYaw());
    poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("PoseEstimator");
  }
}
