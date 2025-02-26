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

        public static final double PIVOT_ZERO_OFFSET = 0.2047741;
        public static final double MAXIMUM_ANGLE = -0.975;
        public static final double ANGLE_TOLERANCE = 0.0; // this is a random default value

        public static final double CORAL_INTAKE_ANGLE = 0.0; // this is a random default value. angle when intaking coral from funnel
        public static final double STOW_ANGLE = -0.2; // this is a random default value. this should be greater than NOCOLLISION_MIN_ARM_ANGLE
        public static final double L1_ANGLE = -0.7; // this is a random default value
        public static final double L2_ANGLE = -0.5; // this is a random default value
        public static final double L3_ANGLE = -0.5; // this is a random default value
        public static final double L4_ANGLE = -0.5; // this is a random default value
    }

    public static class ElevatorConstants {
        public static final int elevatorMotor1_ID = 10;
        public static final int elevatorMotor2_ID = 11;

        public static final double HEIGHT_TOLERANCE = 0.0; // this is a random default value

        public static final double STOW_HEIGHT = 0.0; // this is a random default value
        public static final double INTAKE_HEIGHT = 0.0; // this is a random default value
        public static final double L1_HEIGHT = 0.0; // this is a random default value
        public static final double L2_HEIGHT = 0.3; // this is a random default value
        public static final double L3_HEIGHT = 0.6; // this is a random default value
        public static final double L4_HEIGHT = 1.0; // this is a random default value
        public static final double MAXMIMUM_HEIGHT = 1.16;
    }

    public static class ArmElevatorSuperconstants {
        // *collision occurs when the arm is near vertical and the elevator moves up, the arm will hit the top edge of the first stage
        public static final double NOCOLLISION_MIN_ARM_ANGLE = -0.15 - 0.05; // -0.15 is the real value, I added 0.05 buffer. minimum angle needed for arm not to collide
        public static final double NOCOLLISION_MAX_ELEVATOR_HEIGHT = 0.42 - 0.1; // 0.42 is the real value, I added 0.1 buffer. max elevator height while not colliding
    }

    public static class ClimberConstants {
        public static final int climberMotorID = 15;
    }

    public static class FunnelConstants {
        public static final int funnelMotorID = 14;
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
        EIGHT_LEFT (new Pose2d(5.291 + 8.57, 5.082, Rotation2d.fromDegrees(-120))),
        EIGHT_RIGHT (new Pose2d(4.995 + 8.57, 5.248, Rotation2d.fromDegrees(-120))),
        NINE_LEFT (new Pose2d(3.978 + 8.57, 5.241, Rotation2d.fromDegrees(-60))),
        NINE_RIGHT (new Pose2d(3.689 + 8.57, 5.072, Rotation2d.fromDegrees(-60))),
        TEN_LEFT (new Pose2d(3.180 + 8.57, 4.192, Rotation2d.fromDegrees(0))),
        TEN_RIGHT (new Pose2d(3.180 + 8.57, 3.860, Rotation2d.fromDegrees(0))),
        ELEVEN_LEFT (new Pose2d(3.688 + 8.57, 2.978, Rotation2d.fromDegrees(60))),
        ELEVEN_RIGHT (new Pose2d(3.979 + 8.57, 2.815, Rotation2d.fromDegrees(60))),
        SIX_LEFT (new Pose2d(4.994 + 8.57, 2.814, Rotation2d.fromDegrees(120))),
        SIX_RIGHT (new Pose2d(5.278 + 8.57, 2.979, Rotation2d.fromDegrees(120))),
        SEVEN_LEFT (new Pose2d(5.790 + 8.57, 3.855, Rotation2d.fromDegrees(180))), 
        SEVEN_RIGHT (new Pose2d(5.790 + 8.57, 4.190, Rotation2d.fromDegrees(180)));

        private final Pose2d pose;
        SCORING_POSES_RED(Pose2d pose) { 
            this.pose = pose;
        }

        public Pose2d getPose() { return pose; }
    }

    public static final Pose2d[] scoringPoses = {
        new Pose2d(5.291, 5.082, Rotation2d.fromDegrees(-120)),
        new Pose2d(4.995, 5.248, Rotation2d.fromDegrees(-120)),
        new Pose2d(3.978, 5.241, Rotation2d.fromDegrees(-60)),
        new Pose2d(3.689, 5.072, Rotation2d.fromDegrees(-60)),
        new Pose2d(3.180, 4.192, Rotation2d.fromDegrees(0)),
        new Pose2d(3.180, 3.860, Rotation2d.fromDegrees(0)),
        new Pose2d(3.688, 2.978, Rotation2d.fromDegrees(60)),
        new Pose2d(3.979, 2.815, Rotation2d.fromDegrees(60)),
        new Pose2d(4.994, 2.814, Rotation2d.fromDegrees(120)),
        new Pose2d(5.278, 2.979, Rotation2d.fromDegrees(120)),
        new Pose2d(5.790, 3.855, Rotation2d.fromDegrees(180)), 
        new Pose2d(5.790, 4.190, Rotation2d.fromDegrees(180)),
        new Pose2d(5.291 + 8.57, 5.082, Rotation2d.fromDegrees(-120)),
        new Pose2d(4.995 + 8.57, 5.248, Rotation2d.fromDegrees(-120)),
        new Pose2d(3.978 + 8.57, 5.241, Rotation2d.fromDegrees(-60)),
        new Pose2d(3.689 + 8.57, 5.072, Rotation2d.fromDegrees(-60)),
        new Pose2d(3.180 + 8.57, 4.192, Rotation2d.fromDegrees(0)),
        new Pose2d(3.180 + 8.57, 3.860, Rotation2d.fromDegrees(0)),
        new Pose2d(3.688 + 8.57, 2.978, Rotation2d.fromDegrees(60)),
        new Pose2d(3.979 + 8.57, 2.815, Rotation2d.fromDegrees(60)),
        new Pose2d(4.994 + 8.57, 2.814, Rotation2d.fromDegrees(120)),
        new Pose2d(5.278 + 8.57, 2.979, Rotation2d.fromDegrees(120)),
        new Pose2d(5.790 + 8.57, 3.855, Rotation2d.fromDegrees(180)), 
        new Pose2d(5.790 + 8.57, 4.190, Rotation2d.fromDegrees(180))
    };

    

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