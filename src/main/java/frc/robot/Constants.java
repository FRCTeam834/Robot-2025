// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static class SwerveConstants {
        
        //TODO: Figure out units and change constants

        //SwerveModule.java
        public static final double MODULE_WHEEL_DIAMETER = Units.inchesToMeters(4); // meters
        public static final double DRIVE_GEAR_RATIO = 8.14; 
        public static final double STEER_GEAR_RATIO = 12.8;

        public static final double drivingVelocityFeedForward = 3.248; // Allegedly 1/473 Kv but 2022 code uses 3.248 

        public static final double CAN_CODER_OFFSET_FL = 0.394; // rotations
        public static final double CAN_CODER_OFFSET_FR = 0.275658;
        public static final double CAN_CODER_OFFSET_BR = 0.193359;
        public static final double CAN_CODER_OFFSET_BL = 0.3125;

        // DriveTrain.java
        public static final double DRIVE_LENGTH = Units.inchesToMeters(30); //m
        public static final double DRIVE_WIDTH = Units.inchesToMeters(30); //m
        public static final double MAX_MODULE_SPEED = 0.5; // 8 // m/s
        public static final double MAX_TRANSLATION_SPEED = 0.5; // 4 // m/s
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
        public static final boolean useVisionPoseEstimator = true;
        public static final boolean useMegatag2 = true;
        public static final double maxDegreesPerSecond = 720;
    }
}
