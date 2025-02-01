// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.utility.PoseEstimator;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final DriveTrain driveTrain;
  private final Gyro gyro;
  
  private LimelightHelpers.PoseEstimate lastEstimate;
  private LimelightHelpers.PoseEstimate estimate;
  private boolean rejectUpdate;

  public Limelight(DriveTrain driveTrain, Gyro gyro) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;

    lastEstimate = new LimelightHelpers.PoseEstimate();
    estimate = new LimelightHelpers.PoseEstimate();

    SmartDashboard.putData(this);
  }

  public void setRobotOrientation(Rotation2d yaw) {
    LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.CAM_ONE_NAME, yaw.getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.CAM_TWO_NAME, yaw.getDegrees(), 0, 0, 0, 0, 0);
  } 

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(Constants.VisionConstants.CAM_TWO_NAME, mode);
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate2d(String cameraName) {
    rejectUpdate = false;

    if (VisionConstants.useMegatag2) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

      if(Math.abs(gyro.getAngularVelo()) > VisionConstants.maxDegreesPerSecond) rejectUpdate = true;
      if(estimate.tagCount == 0) rejectUpdate = true;
    } else {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.VisionConstants.CAM_ONE_NAME);

      if(estimate.tagCount == 1 && estimate.rawFiducials.length == 1) {
        if (estimate.rawFiducials[0].ambiguity > 0.7) rejectUpdate = true;
        if (estimate.rawFiducials[0].distToCamera > 3) rejectUpdate = true;
      }
      if(estimate.tagCount == 0) rejectUpdate = true;
    }

    if(rejectUpdate) return lastEstimate;

    lastEstimate = estimate;
    return estimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
