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
        public static final double MODULE_WHEEL_DIAMETER = Units.inchesToMeters(3.95); // meters
        public static final double DRIVE_GEAR_RATIO = 8.14; 
        public static final double STEER_GEAR_RATIO = 12.8;

<<<<<<< HEAD
        public static final double drivingVelocityFeedForward = 1/473; // Allegedly 1/Motor Kv but 2022 code uses 3.248 

        public static final double CAN_CODER_OFFSET_FL = 0;
        public static final double CAN_CODER_OFFSET_FR = 0;
        public static final double CAN_CODER_OFFSET_BL = 0;
        public static final double CAN_CODER_OFFSET_BR = 0;

        // DriveTrain.java
        public static final double DRIVE_LENGTH = 0; //m
        public static final double DRIVE_WIDTH = 0; //m
=======
        public static final double drivingVelocityFeedForward = 3.248; // Allegedly 1/473 Kv but 2022 code uses 3.248 

        public static final double CAN_CODER_OFFSET_FL = -0.286; // rotations
        public static final double CAN_CODER_OFFSET_FR = -0.454;
        public static final double CAN_CODER_OFFSET_BL = 0.058;
        public static final double CAN_CODER_OFFSET_BR = -0.1213;

        // DriveTrain.java
        public static final double DRIVE_LENGTH = Units.inchesToMeters(30); //m
        public static final double DRIVE_WIDTH = Units.inchesToMeters(30); //m
>>>>>>> 69ab3b2 (test)
        public static final double MAX_MODULE_SPEED = 8; // m/s
        public static final double MAX_TRANSLATION_SPEED = 4; // m/s
        public static final double MAX_STEER_SPEED = Units.degreesToRadians(180);

        public static final Translation2d[] moduleTranslations = {
            new Translation2d(DRIVE_LENGTH / 2, DRIVE_WIDTH / 2),
            new Translation2d(DRIVE_LENGTH / 2, -DRIVE_WIDTH / 2),
            new Translation2d(-DRIVE_LENGTH / 2, DRIVE_WIDTH / 2),
            new Translation2d(-DRIVE_LENGTH / 2, -DRIVE_WIDTH / 2)
        };

        //Gyro.java
        //TODO
<<<<<<< HEAD
        public static final int PIGEON_ID = 0;
=======
        public static final int PIGEON_ID = 18;
>>>>>>> 69ab3b2 (test)
    }
}
