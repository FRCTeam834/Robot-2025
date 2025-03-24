// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
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
import frc.robot.utility.LEDs;
import frc.robot.utility.LEDs.ledColor;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BetterAutoDrive extends Command {

  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;
  private final LEDs leds;

  private final Limelight cam_left = RobotContainer.cams[0];
  private final Limelight cam_right = RobotContainer.cams[1];

  private Pose2d[] scoringPosesBlue = new Pose2d[12];
  private Pose2d[] scoringPosesRed = new Pose2d[12];

  PIDController xController = new PIDController(2, 0, 0);
  PIDController yController = new PIDController(2, 0, 0);
  PIDController thetaController = new PIDController(2.5, 0, 0);

  private boolean isRed;
  private String stage;

  private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
    xController, yController,
    new ProfiledPIDController(0, 0, 0,
     new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)))
  );    

  private Pose2d closestScoringPose;
  private Pose2d lineupScoringPose;
  private int closestTagID;
  private double desiredLinearVelocity;
  private String scoringSide; // "left" | "right"
  
  public BetterAutoDrive(String scoringSide, DriveTrain driveTrain, PoseEstimator poseEstimator, LEDs leds) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;
    this.leds = leds;
    this.scoringSide = scoringSide;

    for(int side = 0; side < 6; side++) {
      scoringPosesBlue[side] = getReefPose(side, -1, false);
      scoringPosesBlue[side+6] = getReefPose(side, 1, false);

      scoringPosesRed[side] = getReefPose(side, -1, true);
      scoringPosesRed[side+6] = getReefPose(side, 1, true);
    }

    thetaController.enableContinuousInput(0, 2 * Math.PI);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setLEDColor(ledColor.RED);
    isRed = false;
    stage = "lineup";
    closestScoringPose = null;
    lineupScoringPose = null;

    Pose2d[] usedPoses = scoringPosesBlue;
    if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      usedPoses = scoringPosesRed;
      isRed = true;
    }

    Pose2d robotPose = poseEstimator.getPoseEstimateNoOffset();

    double bestCost = Double.MAX_VALUE;
    for(int i = 0; i < usedPoses.length; i++) {
      if (usedPoses[i] == null) continue;

      double distanceError = robotPose.getTranslation().getDistance(usedPoses[i].getTranslation());
      double angleError = Math.abs(robotPose.getRotation().minus(usedPoses[i].getRotation()).getRadians());

      double cost = (distanceError * 3 + angleError * 2);
      if (cost < bestCost) {
        bestCost = cost;
        closestScoringPose = usedPoses[i]; 
        closestTagID = isRed ? FieldConstants.REEF_TAGS_RED[i] : FieldConstants.REEF_TAGS_BLUE[i];
      }
    }

    if (closestScoringPose == null) end(true);

    System.out.println("Closest tag id: " + closestTagID);
    // cam_left.setTagsFilter(new int[]{closestTagID});
    // cam_right.setTagsFilter(new int[]{closestTagID});

    double dx = -Math.cos(closestScoringPose.getRotation().getRadians()) * FieldConstants.LINEUP_DISTANCE;
    double dy = -Math.sin(closestScoringPose.getRotation().getRadians()) * FieldConstants.LINEUP_DISTANCE;
    Translation2d lineupTranslation = closestScoringPose.getTranslation().plus(new Translation2d(dx, dy));
    lineupScoringPose = new Pose2d(lineupTranslation, closestScoringPose.getRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = poseEstimator.getPoseEstimateNoOffset();
    // double translationToLineupError = robotPose.getTranslation().getDistance(lineupScoringPose.getTranslation());
    double translationToScorePoseError = robotPose.getTranslation().getDistance(closestScoringPose.getTranslation());

    
    ChassisSpeeds speeds = new ChassisSpeeds();
    if(stage.equals("lineup")) {
      if(poseEstimator.isAtPose(lineupScoringPose, Units.inchesToMeters(4), Units.degreesToRadians(5))) {
        stage = "score";
      } else {
        desiredLinearVelocity = 0.2;
        speeds = holonomicDriveController.calculate(
          poseEstimator.getPoseEstimateNoOffset(),
          lineupScoringPose,
          desiredLinearVelocity, 
          closestScoringPose.getRotation()
        );
      }
    }

    if(stage.equals("score")) {
      if (translationToScorePoseError > 0.05) {
        desiredLinearVelocity = 0.075;
      } else {
        desiredLinearVelocity = 0.0;
      }
      speeds = holonomicDriveController.calculate(
        poseEstimator.getPoseEstimateNoOffset(),
        closestScoringPose,
        desiredLinearVelocity, 
        closestScoringPose.getRotation()
      );
    }

    // ChassisSpeeds speeds = holonomicDriveController.calculate(
    //     poseEstimator.getPoseEstimateNoOffset(),
    //     closestScoringPose,
    //     desiredLinearVelocity, 
    //     closestScoringPose.getRotation()
    //   );

    speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -1, 1);
    speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -1, 1);
    speeds.omegaRadiansPerSecond = thetaController.calculate(poseEstimator.getPoseEstimateNoOffset().getRotation().getRadians(), closestScoringPose.getRotation().getRadians());
    driveTrain.setDesiredSpeeds(speeds);
    //SmartDashboard.putNumber("AUTOALIGN ROTATION ERROR", Math.abs(poseEstimator.getPoseEstimateNoOffset().getRotation().getRadians() - closestScoringPose.getRotation().getRadians()));
    //SmartDashboard.putNumber("TRANSLATION ERROR", translationError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setColorForTime(ledColor.GREEN, 1.25);
    driveTrain.stop();
    
    cam_left.resetTagsFilter();
    cam_right.resetTagsFilter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(closestScoringPose.getX() - poseEstimator.getPoseEstimateNoOffset().getX()) < Units.inchesToMeters(0.5))
    && (Math.abs(closestScoringPose.getY() - poseEstimator.getPoseEstimateNoOffset().getY()) < Units.inchesToMeters(0.5))
    && (Math.abs(poseEstimator.getPoseEstimateNoOffset().getRotation().getDegrees() - closestScoringPose.getRotation().getDegrees()) < 2);
  }

  /**
   * Calculates the pose of the robot for scoring on a branch or trough.
   *
   * @param side The side of the reef (0 for left, increases clockwise).
   * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
   * @param flipToRed Red side reef
   * @return The calculated Pose2d for scoring.
   */
  private Pose2d getReefPose(int side, int relativePos, boolean flipToRed) {
    if (
      scoringSide.equals("left") &&
      FieldConstants.scoringSideLeft.containsKey(Rotation2d.fromDegrees(-60 * side))
    ) {
      if ((int)FieldConstants.scoringSideLeft.get(Rotation2d.fromDegrees(-60 * side)) != relativePos) {
        return null;
      }
    }

    if (
      scoringSide.equals("right") &&
      FieldConstants.scoringSideRight.containsKey(Rotation2d.fromDegrees(-60 * side))
    ) {
      if ((int)FieldConstants.scoringSideRight.get(Rotation2d.fromDegrees(-60 * side)) != relativePos) {
        return null;
      }
    }

    // initially do all calculations from blue, then flip later
    Translation2d reefCenter = FieldConstants.REEF_CENTER_BLUE;

    // robot position centered on close reef side
    Translation2d translation = reefCenter.plus(new Translation2d(FieldConstants.REEF_APOTHEM.unaryMinus(), Meters.zero()));
    // translate to correct branch (left, right, cent er)
    translation = translation.plus(FieldConstants.CENTERED_TO_LEFT_BRANCH.times(relativePos));
    // rotate to correct side
    translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side));

    // make pose from translation and correct rotation
    Pose2d reefPose = new Pose2d(translation, Rotation2d.fromDegrees(-60 * side));

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
}
