// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.PoseEstimator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BetterAutoDrive extends Command {
  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;

  private Pose2d closestScoringPose;

  private ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 0.5));
  private ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 0.5));
  private PIDController thetaController = new PIDController(1, 0, 0);

  private double strengthThreshold = Units.feetToMeters(4);

  public BetterAutoDrive(DriveTrain driveTrain, PoseEstimator poseEstimator) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;

    thetaController.enableContinuousInput(0, 2 * Math.PI);
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

    xController.setP(1);
    yController.setP(1);

    ChassisSpeeds currentSpeeds = driveTrain.getRobotRelativeSpeeds();
    xController.reset(robotPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(robotPose.getY(), currentSpeeds.vyMetersPerSecond);

    xController.setGoal(new TrapezoidProfile.State(closestScoringPose.getX(), 0));
    yController.setGoal(new TrapezoidProfile.State(closestScoringPose.getY(), 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = poseEstimator.getPoseEstimate();
    double dist = robotPose.getTranslation().getDistance(closestScoringPose.getTranslation());

    // if (dist < strengthThreshold) {
    //   xController.setP(0.5);
    //   yController.setP(0.5);
    // }

    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
      xController.calculate(robotPose.getX()), 
      yController.calculate(robotPose.getY()),
      thetaController.calculate(driveTrain.getYaw().minus(closestScoringPose.getRotation()).getRadians())
    );

    driveTrain.setDesiredSpeeds(desiredSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(closestScoringPose.getX() - poseEstimator.getPoseEstimate().getX()) < Units.inchesToMeters(1))
    && (Math.abs(closestScoringPose.getY() - poseEstimator.getPoseEstimate().getY()) < Units.inchesToMeters(1));
  }
}
