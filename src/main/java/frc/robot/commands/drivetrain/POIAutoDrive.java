// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.awt.Color;
import java.util.List;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utility.PoseEstimator;
import frc.robot.utility.LEDs;
import frc.robot.utility.LEDs.ledColor;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class POIAutoDrive extends Command {

  private final DriveTrain driveTrain;
  private final Limelight limelight;
  private final LEDs leds;
  private final int relativePos;

  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController thetaController = new PIDController(2.5, 0, 0);

  private Pose2d closestScoringPose;
  private Translation2d tagToBranchTranslation;
  private Rotation2d scoringAngle;

  // relativePos: 1 right / -1 left
  public POIAutoDrive(int relativePos, DriveTrain driveTrain, Limelight limelight, LEDs leds) {
    this.driveTrain = driveTrain;
    this.limelight = limelight;
    this.leds = leds;
    this.relativePos = relativePos;

    tagToBranchTranslation = new Translation2d(VisionConstants.POI_XOFFSET * relativePos, 0);

    thetaController.enableContinuousInput(0, 2 * Math.PI);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setLEDColor(ledColor.RED);
    scoringAngle = FieldConstants.APRIL_TAGS.getTagPose((int)limelight.getCurrentTagID()).get().getRotation().toRotation2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d botToBranchTranslation = limelight.getBotTranslationTargetSpace().plus(tagToBranchTranslation);

    speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -0.8, 0.8);
    speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -0.8, 0.8);
    speeds.omegaRadiansPerSecond = thetaController.calculate(poseEstimator.getPoseEstimateNoOffset().getRotation().getRadians(), closestScoringPose.getRotation().getRadians());
    driveTrain.setDesiredSpeeds(speeds);
    //SmartDashboard.putNumber("AUTOALIGN ROTATION ERROR", Math.abs(poseEstimator.getPoseEstimateNoOffset().getRotation().getRadians() - closestScoringPose.getRotation().getRadians()));
    //SmartDashboard.putNumber("TRANSLATION ERROR", translationError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended");
    leds.setColorForTime(ledColor.GREEN, 1.25);
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(closestScoringPose.getX() - poseEstimator.getPoseEstimateNoOffset().getX()) < Units.inchesToMeters(0.5))
    && (Math.abs(closestScoringPose.getY() - poseEstimator.getPoseEstimateNoOffset().getY()) < Units.inchesToMeters(0.5));
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
}
