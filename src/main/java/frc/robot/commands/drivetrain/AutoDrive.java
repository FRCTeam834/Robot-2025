// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.PoseEstimator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {

  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;

  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController thetaController = new PIDController(2, 0, 0);

  private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    xController, yController,
    new ProfiledPIDController(0, 0, 0,
     new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)))
  );

  private Pose2d closestScoringPose;
  private double desiredLinearVelocity;

  public AutoDrive(DriveTrain driveTrain, PoseEstimator poseEstimator) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;

    thetaController.enableContinuousInput(0, 2 * Math.PI);
    holonomicDriveController.setTolerance(new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(2)));
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Pose2d robotPose = poseEstimator.getPoseEstimate();

    double bestCost = Double.MAX_VALUE;
    for(Pose2d scoringPose : Constants.updatedScoringPoses) {
      double distanceError = robotPose.getTranslation().getDistance(scoringPose.getTranslation());
      double angleError = Math.abs(robotPose.getRotation().minus(scoringPose.getRotation()).getRadians());

      double cost = (distanceError * 3 + angleError * 2);
      if (cost < bestCost) {
        bestCost = cost;
        closestScoringPose = scoringPose; 
      }
    }

    System.out.println(closestScoringPose.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = poseEstimator.getPoseEstimate();
    double translationError = robotPose.getTranslation().getDistance(closestScoringPose.getTranslation());

    if(translationError > 1) {
      desiredLinearVelocity = 0.5;
      xController.setP(1);
      yController.setP(1);
    } else {
      yController.setP(1.5);
      xController.setP(1.5);
      desiredLinearVelocity = 0;
    }

    ChassisSpeeds speeds = holonomicDriveController.calculate(
      new Pose2d(poseEstimator.getPoseEstimate().getTranslation(), driveTrain.getYaw()),
      new Pose2d(closestScoringPose.getTranslation(), new Rotation2d()),
      desiredLinearVelocity, 
      closestScoringPose.getRotation()
    );

    speeds.omegaRadiansPerSecond = thetaController.calculate(driveTrain.getYaw().getRadians(), closestScoringPose.getRotation().getRadians());
    driveTrain.setDesiredSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended");
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(closestScoringPose.getX() - poseEstimator.getPoseEstimate().getX()) < Units.inchesToMeters(1))
    && (Math.abs(closestScoringPose.getY() - poseEstimator.getPoseEstimate().getY()) < Units.inchesToMeters(1));
    //return holonomicDriveController.atReference();
  }
}
