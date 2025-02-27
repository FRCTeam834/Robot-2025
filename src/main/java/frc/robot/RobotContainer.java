// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ArmElevatorGotoPosition;
import frc.robot.commands.arm.DumbArm;
import frc.robot.commands.arm.IntakeCoral;
import frc.robot.commands.arm.OuttakeCoral;
import frc.robot.commands.arm.TestArmPID;
import frc.robot.commands.arm.TuneArm;
import frc.robot.commands.drivetrain.DriveToPose;
import frc.robot.commands.drivetrain.DriveWithSpeeds;
import frc.robot.commands.drivetrain.RotateToPathTarget;
import frc.robot.commands.elevator.DumbElevator;
import frc.robot.commands.elevator.TuneElevator;
import frc.robot.commands.elevator.testElevatorPID;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utility.PoseEstimator;

public class RobotContainer {

  JoystickButton leftJoystick3 = new JoystickButton(OI.leftJoystick, 3);
  JoystickButton leftJoystick10 = new JoystickButton(OI.leftJoystick, 10);
  JoystickButton leftJoystick11 = new JoystickButton(OI.leftJoystick, 11);
  JoystickButton leftJoystick7 = new JoystickButton(OI.leftJoystick, 7);
  JoystickButton leftJoystick6 = new JoystickButton(OI.leftJoystick, 6);

  JoystickButton aButton = new JoystickButton(OI.xbox, 1);
  JoystickButton bButton = new JoystickButton(OI.xbox, 2);
  JoystickButton xButton = new JoystickButton(OI.xbox, 3);
  JoystickButton yButton = new JoystickButton(OI.xbox, 4);

  private static SwerveModule FL = new SwerveModule(3, 2, 20, SwerveConstants.CAN_CODER_OFFSET_FL, false);
  private static SwerveModule FR = new SwerveModule(5, 4, 21, SwerveConstants.CAN_CODER_OFFSET_FR, true);
  private static SwerveModule BL = new SwerveModule(9, 8, 22, SwerveConstants.CAN_CODER_OFFSET_BL, false);
  private static SwerveModule BR = new SwerveModule(7, 6, 23, SwerveConstants.CAN_CODER_OFFSET_BR, true);

  public static Gyro gyro = new Gyro();
  public static DriveTrain driveTrain = new DriveTrain(
    FL, FR, BL, BR,
    gyro
  );


  private static Limelight[] cams = {
    new Limelight(Constants.VisionConstants.CAM_FRONT_NAME, true, driveTrain, gyro), 
    new Limelight(Constants.VisionConstants.CAM_BACK_NAME, true, driveTrain, gyro)
  };
  public static PoseEstimator estimator = new PoseEstimator(driveTrain, cams);
  public static Elevator elevator = new Elevator();
  public static Arm arm = new Arm();

  public RobotContainer() {
    driveTrain.setDefaultCommand(new DriveWithSpeeds(
      driveTrain,
      OI::getRightJoystickX,
      OI::getRightJoystickY,
      OI::getLeftJoystickX
    ));
    driveTrain.configureAutoBuilder(estimator);

    //elevator.setDefaultCommand(new TuneElevator(elevator));
    //arm.setDefaultCommand(new TuneArm(arm));

    configureBindings();

    //elevator.setDefaultCommand(new DumbElevator(elevator, OI::getXboxLeftJoystickY));
    //arm.setDefaultCommand(new DumbArm(arm, OI::getXboxRightJoystickY));
    configureTestingBindings();
  }

  private void configureTestingBindings() {
    // aButton.onTrue(new TestArmPID(arm, -0.3));
    // bButton.onTrue(new TestArmPID(arm, -0.5));
    // xButton.onTrue(new TestArmPID(arm, -0.7));
    // yButton.onTrue(new TestArmPID(arm, -0.1));

    //aButton.onTrue(new testElevatorPID(elevator, 0.2));
    //bButton.onTrue(new testElevatorPID(elevator, 0.5));
    //xButton.onTrue(new testElevatorPID(elevator, 0.8));
    //yButton.onTrue(new testElevatorPID(elevator, 1));
    
    aButton.onTrue(new ArmElevatorGotoPosition(-0.21, 1.38, arm, elevator));
    //bButton.onTrue(new ArmElevatorGotoPosition(-0.3, 0.7, arm, elevator));
    xButton.onTrue(new ArmElevatorGotoPosition(0.5, 0.0, arm, elevator));
    yButton.whileTrue(new IntakeCoral(arm));

    bButton.whileTrue(new OuttakeCoral(arm));
  }

  private void configureBindings() {
    leftJoystick10.onTrue(new InstantCommand(() -> {
      driveTrain.zeroOdometry(new Rotation2d());
      cams[1].seedLL4IMU();

      FL.resetDriveEncoder();
      FR.resetDriveEncoder();
      BL.resetDriveEncoder();
      BR.resetDriveEncoder();

      System.out.println("Zeroed the odometry");
    }));

    leftJoystick3.onTrue(new InstantCommand(() -> {
      FL.zeroCANCoder();
      FR.zeroCANCoder();
      BL.zeroCANCoder();
      BR.zeroCANCoder();
      System.out.println("Updated CANCoder zero");
    }));

    
    leftJoystick11.onTrue(new InstantCommand(() -> {
      FL.seedTurnEncoder();
      FR.seedTurnEncoder();
      BL.seedTurnEncoder();
      BR.seedTurnEncoder();
    }));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
