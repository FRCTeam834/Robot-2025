// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.Gyro;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final Gyro gyro;
  private final String cameraName;

  public Limelight(String cameraName, Gyro gyro) {
    this.cameraName = cameraName;
    this.gyro = gyro;

    SmartDashboard.putData(this);
  }

  public void setRobotOrientation(Rotation2d yaw) {
    LimelightHelpers.SetRobotOrientation(cameraName, yaw.getDegrees(), 0, 0, 0, 0, 0);
  } 

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(cameraName, mode);
  }

  public Translation2d getBotTranslation2d_TargetSpace() {
    Pose3d botPose = LimelightHelpers.getBotPose3d_TargetSpace(cameraName);
    Translation2d convertedTranslation = new Translation2d(botPose.getX(), botPose.getZ());
    return convertedTranslation;
  }

  public double getCurrentTagID() {
    return LimelightHelpers.getFiducialID(cameraName);
  }

  public BooleanSupplier hasTarget() {
    return () -> LimelightHelpers.getTargetCount(cameraName) != 0;
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate2d() {
    boolean rejectUpdate = false;
    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if(estimate == null) return null;

    if(Math.abs(gyro.getAngularVelo()) > VisionConstants.maxDegreesPerSecond) rejectUpdate = true;
    if(estimate.tagCount == 0) rejectUpdate = true;

    for(LimelightHelpers.RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial.distToRobot > 3) rejectUpdate = true;
    }

    if (rejectUpdate) return null;
    return estimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
