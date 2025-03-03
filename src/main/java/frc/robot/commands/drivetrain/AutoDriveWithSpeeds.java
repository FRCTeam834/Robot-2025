// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.PoseEstimator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDriveWithSpeeds extends Command {

  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;

  private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    new PIDController(2, 0, 0), new PIDController(2, 0, 0), 
    new ProfiledPIDController(3, 0, 0,
     new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)))
  );

  private Pose2d closestScoringPose;

  private Trajectory desiredTrajectory;
  private TrajectoryConfig trajectoryConfig;
  private Timer driveTimer = new Timer();

  public AutoDriveWithSpeeds(DriveTrain driveTrain, PoseEstimator poseEstimator) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;

    holonomicDriveController.setTolerance(new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(2)));

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTimer.stop();
    driveTimer.reset();

    Pose2d robotPose = poseEstimator.getPoseEstimate();

    double bestCost = Double.MAX_VALUE;
    for(Pose2d scoringPose : Constants.scoringPoses) {
      double distanceError = robotPose.getTranslation().getDistance(scoringPose.getTranslation());
      double angleError = Math.abs(robotPose.getRotation().minus(scoringPose.getRotation()).getRadians());

      double cost = (distanceError * 0.5 + angleError * 5);
      if (cost < bestCost) {
        bestCost = cost;
        closestScoringPose = scoringPose; 
      }
    }

    trajectoryConfig = new TrajectoryConfig(2, 2);
    trajectoryConfig.setStartVelocity(driveTrain.getRobotVeloMagnitude());  
    trajectoryConfig.setKinematics(driveTrain.getKinematics());

    desiredTrajectory = TrajectoryGenerator.generateTrajectory(
      robotPose, 
      null, 
      closestScoringPose, 
      trajectoryConfig);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTimer.start();
    ChassisSpeeds speeds = holonomicDriveController.calculate(
      poseEstimator.getPoseEstimate(), 
      desiredTrajectory.sample(driveTimer.get()),
      closestScoringPose.getRotation()
    );

    driveTrain.setDesiredSpeedsFromHolonomicController(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTimer.stop();
    driveTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return holonomicDriveController.atReference();
  }
}
