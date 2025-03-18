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
public class AutoDriveWithNewVision extends Command {

  private final DriveTrain driveTrain;
  private final Limelight limelight;
  private final LEDs leds;
  private final PoseEstimator estimator;

  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController thetaController = new PIDController(2.5, 0, 0);

  private Translation2d tagToBranchTranslation;
  private Translation2d botToBranchTranslation;
  private Rotation2d scoringAngle;

  // relativePos: 1 right / -1 left
  public AutoDriveWithNewVision(int relativePos, DriveTrain driveTrain, Limelight limelight, PoseEstimator estimator, LEDs leds) {
    this.driveTrain = driveTrain;
    this.limelight = limelight;
    this.leds = leds;
    this.estimator = estimator;

    tagToBranchTranslation = new Translation2d(VisionConstants.BRANCH_XOFFSET * relativePos, 0);

    thetaController.enableContinuousInput(0, 2 * Math.PI);
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setLEDColor(ledColor.RED);
    scoringAngle = FieldConstants.APRIL_TAGS.getTagPose((int)limelight.getCurrentTagID()).get().getRotation().toRotation2d();
    System.out.println(scoringAngle.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    botToBranchTranslation = limelight.getBotTranslation2d_TargetSpace().plus(tagToBranchTranslation);

    double vx = xController.calculate(botToBranchTranslation.getX());
    double vy = yController.calculate(botToBranchTranslation.getY());

    ChassisSpeeds speeds = new ChassisSpeeds();
    speeds.vxMetersPerSecond = MathUtil.clamp(vx, -1, 1);
    speeds.vyMetersPerSecond = MathUtil.clamp(vy, -1, 1);
    speeds.omegaRadiansPerSecond = thetaController.calculate(estimator.getPoseEstimateNoOffset().getRotation().getRadians(), scoringAngle.getRadians());
    driveTrain.setDesiredSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setColorForTime(ledColor.GREEN, 1.25);
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(Units.metersToInches(botToBranchTranslation.getX())) < 0.5 
    && Math.abs(Units.metersToInches(botToBranchTranslation.getY())) < 0.5);
  }
}
