// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ArmElevatorGotoPosition;
import frc.robot.commands.CoralIntakeSequence;
import frc.robot.commands.arm.DumbArm;
import frc.robot.commands.arm.IntakeAlgae;
import frc.robot.commands.arm.IntakeCoral;
import frc.robot.commands.arm.OuttakeCoral;
import frc.robot.commands.arm.ReverseIntake;
import frc.robot.commands.arm.TestArmPID;
import frc.robot.commands.arm.TuneArm;
import frc.robot.commands.auton.AutonGotoL2;
import frc.robot.commands.auton.AutonGotoL4;
import frc.robot.commands.auton.AutonIntake;
import frc.robot.commands.auton.AutonIntakeDos;
import frc.robot.commands.auton.AutonIntakeUno;
import frc.robot.commands.auton.AutonIntakeUnoArm;
import frc.robot.commands.auton.AutonScoreL4;
import frc.robot.commands.drivetrain.AutoDriveWithNewVision;
import frc.robot.commands.drivetrain.BetterAutoDrive;
import frc.robot.commands.drivetrain.DriveWithSpeeds;
import frc.robot.commands.drivetrain.OpenloopDrive;
import frc.robot.commands.drivetrain.RotateToPathTarget;
import frc.robot.commands.elevator.DumbElevator;
import frc.robot.commands.elevator.TuneElevator;
import frc.robot.commands.elevator.testElevatorPID;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utility.PoseEstimator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.commands.climb.ManualClimb;
import frc.robot.commands.climb.ManualFunnel;
import frc.robot.utility.LEDs;
import frc.robot.Constants.*;
import frc.robot.Constants.FieldConstants;

public class RobotContainer {

  JoystickButton leftJoystick3 = new JoystickButton(OI.leftJoystick, 3);
  JoystickButton leftJoystick10 = new JoystickButton(OI.leftJoystick, 10);
  JoystickButton leftJoystick11 = new JoystickButton(OI.leftJoystick, 11);
  JoystickButton leftJoystick7 = new JoystickButton(OI.leftJoystick, 7);
  JoystickButton leftJoystick6 = new JoystickButton(OI.leftJoystick, 6);
  JoystickButton rightJoystick1 = new JoystickButton(OI.rightJoystick, 1);
  JoystickButton leftJoystick1 = new JoystickButton(OI.leftJoystick, 1);

  JoystickButton rightJoystick3 = new JoystickButton(OI.leftJoystick, 3);

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


  public static Limelight[] cams = {
    new Limelight(Constants.VisionConstants.CAM_LEFT_NAME, gyro), 
    new Limelight(Constants.VisionConstants.CAM_RIGHT_NAME, gyro)
  };
  public static PoseEstimator estimator = new PoseEstimator(driveTrain, cams);
  public static Elevator elevator = new Elevator();
  public static Arm arm = new Arm();
  public static Climber climber = new Climber();
  public static Funnel funnel = new Funnel();
  public static LEDs leds = new LEDs();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    driveTrain.setDefaultCommand(new OpenloopDrive(
      driveTrain,
      OI::getRightJoystickX,
      OI::getRightJoystickY,
      OI::getLeftJoystickX
    ));

    driveTrain.configureAutoBuilder(estimator);

    NamedCommands.registerCommand("AutonScoreL4", new AutonScoreL4(driveTrain, estimator, arm, elevator));
    NamedCommands.registerCommand("AutonGotoL4", new AutonGotoL4(elevator, arm));
    NamedCommands.registerCommand("AutonGotoL2", new AutonGotoL2(elevator, arm));
    NamedCommands.registerCommand("AutonIntake", new AutonIntake(arm, elevator, leds));
    NamedCommands.registerCommand("AutonIntakeUno", new AutonIntakeUno(arm, elevator));
    NamedCommands.registerCommand("AutonIntakeDos", new AutonIntakeDos(arm));


    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("LEFT_Top-60L-120R-120L", new PathPlannerAuto("Top-60L-120R-120L"));
    autoChooser.addOption("RIGHT_Top-60L-120R-120L", new PathPlannerAuto("Top-60L-120R-120L", true));
    autoChooser.addOption("Top-60L-120R-120L-AutoDrive", new PathPlannerAuto("Top-60L-120R-120L-AutoDrive"));
    autoChooser.addOption("twopieceleft", new PathPlannerAuto("twopiece"));
    autoChooser.addOption("twopieceright", new PathPlannerAuto("twopiece", true));
    autoChooser.addOption("onepiecemiddle", new PathPlannerAuto("onepiecemiddle"));
    autoChooser.addOption("twol4pieceleft", new PathPlannerAuto("twol4piece"));
    autoChooser.addOption("twol4pieceright", new PathPlannerAuto("twol4piece", true));

    SmartDashboard.putData(autoChooser);


    //elevator.setDefaultCommand(new TuneElevator(elevator));
    //arm.setDefaultCommand(new TuneArm(arm));

    configureBindings();
    //elevator.setDefaultCommand(new DumbElevator(elevator, OI::getXboxLeftJoystickY));
    //arm.setDefaultCommand(new DumbArm(arm, OI::getXboxRightJoystickY));
    //configureTestingBindings();

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
    
    //aButton.onTrue(new ArmElevatorGotoPosition(ArmConstants.L1_ANGLE, ElevatorConstants.L1_HEIGHT, arm, elevator));
    //xButton.onTrue(new ArmElevatorGotoPosition(0.2, 0, arm, elevator));
    //xButton.onTrue(new ArmElevatorGotoPosition(ArmConstants.L3_ANGLE, ElevatorConstants.L3_HEIGHT, arm, elevator));
    //yButton.onTrue(new ArmElevatorGotoPosition(ArmConstants.L4_ANGLE, ElevatorConstants.L4_HEIGHT, arm, elevator));

    //aButton.onTrue(new IntakeCoral(arm));
    //bButton.onTrue(new OuttakeCoral(arm));

    // leftJoystick3.onTrue(new InstantCommand(() -> {
    //   FL.zeroCANCoder();
    //   FR.zeroCANCoder();
    //   BL.zeroCANCoder();
    //   BR.zeroCANCoder();
    //   System.out.println("Updated CANCoder zero");
    // }));
  }

  private void configureBindings() {
    // stow
    OI.getKeypad0().onTrue(new ArmElevatorGotoPosition(ArmConstants.STOW_ANGLE, ElevatorConstants.STOW_HEIGHT, arm, elevator, driveTrain));

    // coral
    OI.getKeypad1().onTrue(new ArmElevatorGotoPosition(ArmConstants.L1_ANGLE, ElevatorConstants.L1_HEIGHT, arm, elevator, driveTrain));
    OI.getKeypad4().onTrue(new ArmElevatorGotoPosition(ArmConstants.L2_ANGLE, ElevatorConstants.L2_HEIGHT, arm, elevator, driveTrain));
    OI.getKeypad7().onTrue(new ArmElevatorGotoPosition(ArmConstants.L3_ANGLE, ElevatorConstants.L3_HEIGHT, arm, elevator, driveTrain));
    OI.getKeypadNum().onTrue(new ArmElevatorGotoPosition(ArmConstants.L4_ANGLE, ElevatorConstants.L4_HEIGHT, arm, elevator, driveTrain));
    OI.getKeypadSlash().onTrue(new ArmElevatorGotoPosition(ArmConstants.L4_CUP_ANGLE, ElevatorConstants.L4_HEIGHT, arm, elevator, driveTrain));

    // algae
    OI.getKeypad2().onTrue(new ArmElevatorGotoPosition(ArmConstants.ALGAE_L1_ANGLE, ElevatorConstants.ALGAE_L1_HEIGHT, arm, elevator, driveTrain));
    OI.getKeypad5().onTrue(new ArmElevatorGotoPosition(ArmConstants.ALGAE_L2_ANGLE, ElevatorConstants.ALGAE_L2_HEIGHT, arm, elevator, driveTrain));

    aButton.onTrue(new CoralIntakeSequence(arm, elevator, leds));
    xButton.onTrue(new IntakeAlgae(arm));
    bButton.onTrue(new OuttakeCoral(arm, elevator));
    yButton.whileTrue(new ReverseIntake(arm));

    // leftJoystick1.whileTrue(new AutoDriveWithNewVision(-1, driveTrain, cams[0], estimator, leds).onlyIf(cams[0].hasTarget()));
    // rightJoystick1.whileTrue(new AutoDriveWithNewVision(1, driveTrain, cams[1], estimator, leds).onlyIf(cams[1].hasTarget()));
    rightJoystick1.whileTrue(new BetterAutoDrive("right", driveTrain, estimator, leds));
    leftJoystick1.whileTrue(new BetterAutoDrive("left", driveTrain, estimator, leds));

    rightJoystick3.onTrue(new ArmElevatorGotoPosition(ArmConstants.CORAL_INTAKE_ANGLE, ElevatorConstants.STOW_HEIGHT, arm, elevator, driveTrain));

    funnel.setDefaultCommand(new ManualFunnel(funnel, OI::getXboxLeftJoystickY));

    leftJoystick10.onTrue(new InstantCommand(() -> {
      estimator.resetRotation(new Rotation2d());

      System.out.println("Zeroed the odometry");
    }));

    
    leftJoystick11.onTrue(new InstantCommand(() -> {
      FL.seedTurnEncoder();
      FR.seedTurnEncoder();
      BL.seedTurnEncoder();
      BR.seedTurnEncoder();
    }));

    climber.setDefaultCommand(new ManualClimb(climber, OI::getXboxRightJoystickY));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
