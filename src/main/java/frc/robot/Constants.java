// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class Constants {
    public static class SwerveConstants {
        
        //TODO: Figure out units and change constants

        //SwerveModule.java
        public static final double MODULE_WHEEL_DIAMETER = Units.inchesToMeters(4); // meters
        public static final double DRIVE_GEAR_RATIO = 8.14; 
        public static final double STEER_GEAR_RATIO = 12.8;

        public static final double drivingVelocityFeedForward = 1.4; // Allegedly 1/473 Kv but 2022 code uses 3.248 

        public static final double CAN_CODER_OFFSET_FL = 0.394; // rotations
        public static final double CAN_CODER_OFFSET_FR = 0.275658;
        public static final double CAN_CODER_OFFSET_BR = 0.193359;
        public static final double CAN_CODER_OFFSET_BL = 0.3125;

        // DriveTrain.java
        public static final double DRIVE_LENGTH = Units.inchesToMeters(30); //m // should be 60cm
        public static final double DRIVE_WIDTH = Units.inchesToMeters(30); //m
        public static final double MAX_MODULE_SPEED = 3; // 8 // m/s
        public static final double MAX_TRANSLATION_SPEED = 2; // 4 // m/s
        public static final double MAX_STEER_SPEED = Units.degreesToRadians(270);

        public static final Translation2d[] moduleTranslations = {
            new Translation2d(DRIVE_WIDTH / 2, DRIVE_LENGTH / 2),
            new Translation2d(DRIVE_WIDTH / 2, -DRIVE_LENGTH / 2),
            new Translation2d(-DRIVE_WIDTH / 2, DRIVE_LENGTH / 2),
            new Translation2d(-DRIVE_WIDTH / 2, -DRIVE_LENGTH / 2)
        };

        //Gyro.java
        //TODO
        public static final int PIGEON_ID = 18;
    }

    public static class VisionConstants {
        public static final boolean useLL4Gyro = true;
        public static final boolean useVisionPoseEstimator = true;
        public static final boolean useMegatag2 = true;
        public static final double maxDegreesPerSecond = 180;

        public static final String CAM_ONE_NAME = "limelight-one"; // 3G
        public static final String CAM_TWO_NAME = "limelight-two"; // 4
        public static final String CAM_THREE_NAME = "limelight-three"; // 4
        
        public static final LimelightStrategies STRATEGY = LimelightStrategies.ALL_ESTIMATES;
    }

    public static AprilTagFieldLayout aprilTagFieldLayout;

    public static enum LimelightStrategies {
        ALL_ESTIMATES,
        BEST_ESTIMATE,
        AVERAGE_ESTIMATE
    }

    public static RobotConfig PATHPLANNER_CONFIG;

    static {
        try {
            PATHPLANNER_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().getPath(), "apriltagLayout.json"));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}