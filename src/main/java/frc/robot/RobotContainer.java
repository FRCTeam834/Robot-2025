// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DriveWithSpeeds;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utility.PoseEstimator;

public class RobotContainer {

  JoystickButton leftJoystick3 = new JoystickButton(OI.leftJoystick, 3);
  JoystickButton leftJoystick10 = new JoystickButton(OI.leftJoystick, 10);
  JoystickButton leftJoystick11 = new JoystickButton(OI.leftJoystick, 11);

  private static SwerveModule FL = new SwerveModule(3, 2, 10, SwerveConstants.CAN_CODER_OFFSET_FL, false);
  private static SwerveModule FR = new SwerveModule(5, 4, 11, SwerveConstants.CAN_CODER_OFFSET_FR, true);
  private static SwerveModule BL = new SwerveModule(9, 8, 13, SwerveConstants.CAN_CODER_OFFSET_BL, false);
  private static SwerveModule BR = new SwerveModule(7, 6, 12, SwerveConstants.CAN_CODER_OFFSET_BR, true);

  public static Gyro gyro = new Gyro();
  public static DriveTrain driveTrain = new DriveTrain(
    FL, FR, BL, BR,
    gyro
  );

  public static Limelight limelight = new Limelight(driveTrain, gyro);
  public static PoseEstimator estimator = new PoseEstimator(driveTrain, limelight);

  public RobotContainer() {
    driveTrain.setDefaultCommand(new DriveWithSpeeds(
      driveTrain,
      OI::getLeftJoystickX,
      OI::getLeftJoystickY,
      OI::getRightJoystickX
    ));

    configureBindings();
  }

  private void configureBindings() {
    leftJoystick10.onTrue(new InstantCommand(() -> {
      driveTrain.zeroOdometry(new Rotation2d());
      System.out.println("Zeroes the odometry");
    }));

    leftJoystick3.onTrue(new InstantCommand(() -> {
      FL.zeroCANCoder();
      FR.zeroCANCoder();
      BL.zeroCANCoder();
      BR.zeroCANCoder();
      limelight.setRobotOrientation(driveTrain.getYaw());
      System.out.println("Zeroed!");
    }));

    leftJoystick11.onTrue(new InstantCommand(() -> {
      FL.seedTurnEncoder();
      FR.seedTurnEncoder();
      BL.seedTurnEncoder();
      BR.seedTurnEncoder();
    }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
