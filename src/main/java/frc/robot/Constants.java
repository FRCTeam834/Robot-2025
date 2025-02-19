// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

        public static final String CAM_FRONT_NAME = "limelight-front"; 
        public static final String CAM_BACK_NAME = "limelight-two"; 
        
        public static final LimelightStrategies STRATEGY = LimelightStrategies.ALL_ESTIMATES;
    }

    public static class ArmConstants {
        public static final int laserCANID = 30;
        public static final int pivotCANID = 12;
        public static final int intakeCANID = 13;
    }

    public static class ElevatorConstants {
        public static final int elevatorMotor1_ID = 10;
        public static final int elevatorMotor2_ID = 11;
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 15;
    }

    public static enum SCORING_POSES_BLUE {
        TWENTY_LEFT (new Pose2d(5.291, 5.082, Rotation2d.fromDegrees(-120))),
        TWENTY_RIGHT (new Pose2d(4.995, 5.248, Rotation2d.fromDegrees(-120))),
        NINETEEN_LEFT (new Pose2d(3.978, 5.241, Rotation2d.fromDegrees(-60))),
        NINETEEN_RIGHT (new Pose2d(3.689, 5.072, Rotation2d.fromDegrees(-60))),
        EIGHTEEN_LEFT (new Pose2d(3.180, 4.192, Rotation2d.fromDegrees(0))),
        EIGHTEEN_RIGHT (new Pose2d(3.180, 3.860, Rotation2d.fromDegrees(0))),
        SEVENTEEN_LEFT (new Pose2d(3.688, 2.978, Rotation2d.fromDegrees(60))),
        SEVENTEEN_RIGHT (new Pose2d(3.979, 2.815, Rotation2d.fromDegrees(60))),
        TWENTYTWO_LEFT (new Pose2d(4.994, 2.814, Rotation2d.fromDegrees(120))),
        TWENTYTWO_RIGHT (new Pose2d(5.278, 2.979, Rotation2d.fromDegrees(120))),
        TWENTYONE_LEFT (new Pose2d(5.790, 3.855, Rotation2d.fromDegrees(180))), 
        TWENTYONE_RIGHT (new Pose2d(5.790, 4.190, Rotation2d.fromDegrees(180)));
        

        private final Pose2d pose;
        SCORING_POSES_BLUE(Pose2d pose) { 
            this.pose = pose;
        }

        public Pose2d getPose() { return pose; }
    }

    public static enum SCORING_POSES_RED {

    }

    public static enum LimelightStrategies {
        ALL_ESTIMATES,
        BEST_ESTIMATE,
        AVERAGE_ESTIMATE
    }

    public static final RobotMode robotMode = RobotMode.DEVELOPMENT;
    public static enum RobotMode {
        COMPETITION,
        DEVELOPMENT
    }

    public static RobotConfig PATHPLANNER_CONFIG;

    static {
        try {
            PATHPLANNER_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}