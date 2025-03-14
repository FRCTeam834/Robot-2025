// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.List;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.utility.PoseEstimator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BetterAutoDrive extends Command {

  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;

  private boolean flipToRed = false;
  private Pose2d[] scoringPoses = new Pose2d[12];

  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController thetaController = new PIDController(2.5, 0, 0);

  private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    xController, yController,
    new ProfiledPIDController(0, 0, 0,
     new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)))
  );

  private Pose2d closestScoringPose;
  private double desiredLinearVelocity;

  public BetterAutoDrive(DriveTrain driveTrain, PoseEstimator poseEstimator) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;

    for(int side = 0; side < 6; side++) {
      scoringPoses[side] = getReefPose(side, -1, flipToRed);
    }
    for(int side = 0; side < 6; side++) {
      scoringPoses[side+6] = getReefPose(side, 1, flipToRed);
    }

    thetaController.enableContinuousInput(0, 2 * Math.PI);
    holonomicDriveController.setTolerance(new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(2), Rotation2d.fromDegrees(2)));
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flipToRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    Pose2d robotPose = poseEstimator.getPoseEstimate();

    double bestCost = Double.MAX_VALUE;
    for(Pose2d scoringPose : scoringPoses) {
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
      xController.setP(2);
      yController.setP(2);
    } else {
      desiredLinearVelocity = 0;
      yController.setP(2);
      xController.setP(2);
    }


    ChassisSpeeds speeds = holonomicDriveController.calculate(
      poseEstimator.getPoseEstimate(),
      closestScoringPose,
      desiredLinearVelocity, 
      closestScoringPose.getRotation()
    );

    speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -1, 1);
    speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -1, 1);
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
    return (Math.abs(closestScoringPose.getX() - poseEstimator.getPoseEstimate().getX()) < Units.inchesToMeters(0.5))
    && (Math.abs(closestScoringPose.getY() - poseEstimator.getPoseEstimate().getY()) < Units.inchesToMeters(0.5));
    //return holonomicDriveController.atReference();
  }

  /**
   * Calculates the pose of the robot for scoring on a branch or trough.
   *
   * @param side The side of the reef (0 for left, increases clockwise).
   * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
   * @return The calculated Pose2d for scoring.
   */
  private Pose2d getReefPose(int side, int relativePos, boolean flipToRed) {
    // determine whether to use red or blue reef position

    // initially do all calculations from blue, then flip later
    Translation2d reefCenter = FieldConstants.REEF_CENTER_BLUE;

    // robot position centered on close reef side
    Translation2d translation = reefCenter.plus(new Translation2d(FieldConstants.REEF_APOTHEM.unaryMinus(), Meters.zero()));
    // translate to correct branch (left, right, cent er)
    translation = translation.plus(FieldConstants.CENTERED_TO_LEFT_BRANCH.times(relativePos));
    // rotate to correct side
    translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

    // make pose from translation and correct rotation
    Pose2d reefPose = new Pose2d(translation,
            Rotation2d.fromDegrees(-60 * side));

    if (flipToRed) {
        reefPose = flipPose(reefPose);
    } 

    return reefPose;
  }

  private Pose2d flipPose(Pose2d pose) {
    Translation2d center = FieldConstants.REEF_CENTER_BLUE.interpolate(FieldConstants.REEF_CENTER_RED, 0.5);
    Translation2d poseTranslation = pose.getTranslation();
    poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
    return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
  }

  public Pose2d[] getBranchPoses() {
    Pose2d[] poses = new Pose2d[12];
    for(int side = 0; side < 6; side++) {
      poses[side] = getReefPose(side, -1, flipToRed);
    }
    for(int side = 0; side < 6; side++) {
      poses[side+6] = getReefPose(side, 1, flipToRed);
    }

    return poses;
  }
}
