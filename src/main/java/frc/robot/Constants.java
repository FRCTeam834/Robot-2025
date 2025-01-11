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
        public static final double drivingFactor = 1 * Math.PI / 1; // wheeldiametermeters * PI / DrivingMotorReduction
        public static final double turningFactor = 2 * Math.PI;
        public static final double drivingVelocityFeedForward = 1; // 1 / DriveFreeWheelSpeedRps

        // DriveTrain.java
        public static final double DRIVE_LENGTH = 0;
        public static final double DRIVE_WIDTH = 0;
        public static final double MAX_MODULE_SPEED = Units.feetToMeters(15);
        public static final double MAX_TRANSLATION_SPEED = Units.feetToMeters(Units.feetToMeters(15));
        public static final double MAX_STEER_SPEED = Units.degreesToRadians(250);

        public static final Translation2d[] moduleTranslations = {
            new Translation2d(DRIVE_LENGTH / 2, DRIVE_WIDTH / 2),
            new Translation2d(DRIVE_LENGTH / 2, -DRIVE_WIDTH / 2),
            new Translation2d(-DRIVE_LENGTH / 2, DRIVE_WIDTH / 2),
            new Translation2d(-DRIVE_LENGTH / 2, -DRIVE_WIDTH / 2)
        };

        //Gyro.java
        //TODO
        public static final int PIGEON_ID = 0;
    }
}
