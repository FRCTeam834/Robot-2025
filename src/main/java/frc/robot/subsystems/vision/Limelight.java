// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.Gyro;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final DriveTrain driveTrain;
  private final Gyro gyro;
  private final String cameraName;
  private final boolean isLL4;

  public Limelight(String cameraName, boolean isLL4, DriveTrain driveTrain, Gyro gyro) {
    this.cameraName = cameraName;
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    this.isLL4 = isLL4;

    if (isLL4) {
      // setIMUMode(1);
      // setRobotOrientation(driveTrain.getYaw());
      // setIMUMode(2);
      setIMUMode(0);
    }

    SmartDashboard.putData(this);
  }

  public void setRobotOrientation(Rotation2d yaw) {
    LimelightHelpers.SetRobotOrientation(cameraName, yaw.getDegrees(), 0, 0, 0, 0, 0);
  } 

  public void setIMUMode(int mode) {
    LimelightHelpers.SetIMUMode(cameraName, mode);
  }

  public void seedLL4IMU() {
    if(!isLL4) return;
    //setIMUMode(1);
    //setRobotOrientation(driveTrain.getYaw());
    //System.out.println("SEEDED LL4");
    //setIMUMode(2);
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate2d() {
    boolean rejectUpdate = false;
    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if(estimate == null) return null;

    if(Math.abs(gyro.getAngularVelo()) > VisionConstants.maxDegreesPerSecond) rejectUpdate = true;
    if(estimate.tagCount == 0) rejectUpdate = true;

    for(LimelightHelpers.RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial.ambiguity > 0.3) rejectUpdate = true;
    }

    if (rejectUpdate) return null;
    return estimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
