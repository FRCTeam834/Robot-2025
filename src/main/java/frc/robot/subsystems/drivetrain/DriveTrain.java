// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.utility.PoseEstimator;
import frc.robot.utility.TunableNumber;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule flSwerveModule;
  private final SwerveModule frSwerveModule;
  private final SwerveModule blSwerveModule;
  private final SwerveModule brSwerveModule;

  private final Gyro gyro;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    SwerveConstants.moduleTranslations[0], 
    SwerveConstants.moduleTranslations[1], 
    SwerveConstants.moduleTranslations[2], 
    SwerveConstants.moduleTranslations[3]
  );

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(1);
  private SlewRateLimiter omegaLimiter = new SlewRateLimiter(Math.toRadians(1080));

  private static TunableNumber kS_TunableNumber = new TunableNumber("SwerveModule/kS");
  private static TunableNumber kV_TunableNumber = new TunableNumber("SwerveModule/kV");

  private static TunableNumber kP_Drive = new TunableNumber("SwerveModule/drivekP");
  private static TunableNumber kP_Turn = new TunableNumber("SwerveModule/turnkP");

  private boolean stopped = false;
  private ChassisSpeeds setpoint = new ChassisSpeeds();

  private Rotation2d keepHeadingAngle = new Rotation2d();
  private PIDController keepHeadingController = new PIDController(2, 0, 0);

  static {
    kS_TunableNumber.initDefault(0.2);
    kV_TunableNumber.initDefault(3);
    kP_Drive.initDefault(0);
    kP_Turn.initDefault(0.17);
  }

  public DriveTrain(
  SwerveModule flSwerveModule,
  SwerveModule frSwerveModule,
  SwerveModule blSwerveModule,
  SwerveModule brSwerveModule,
  Gyro gyro) {
    this.flSwerveModule = flSwerveModule;
    this.frSwerveModule = frSwerveModule;
    this.blSwerveModule = blSwerveModule;
    this.brSwerveModule = brSwerveModule;
    this.gyro = gyro;

    keepHeadingAngle = getYaw();
    SmartDashboard.putData(this);
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public void resetYaw(double angle) {
    gyro.resetYaw(angle);
  }

  public double getYawRadians () {
    return getYaw().getRadians();
  }

  public double getYawDegrees () {
    return getYaw().getDegrees();
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    // double angle = Math.atan2(ySpeed, xSpeed);
    // double mag = translationLimiter.calculate(Math.hypot(xSpeed, ySpeed));
    // xSpeed = mag * Math.cos(angle);
    // ySpeed = mag * Math.sin(angle);
    // rot = omegaLimiter.calculate(rot);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw());

    //speeds = keepHeadingOutput(speeds);
    setDesiredSpeeds(speeds);
  }

  public void setDesiredSpeeds(ChassisSpeeds speeds) {
    stopped = false;
    setpoint = speeds;
  }

  public void setDesiredSpeedsFromHolonomicController(ChassisSpeeds speeds) {
    double angle = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
    double mag = translationLimiter.calculate(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    speeds.vxMetersPerSecond = mag * Math.cos(angle);
    speeds.vyMetersPerSecond = mag * Math.sin(angle);
    speeds.omegaRadiansPerSecond = omegaLimiter.calculate(speeds.omegaRadiansPerSecond);

    stopped = false;
    setpoint = speeds;
  }

  public void setChassisSlewRate(double translationLimit, double omegaLimit) {
    double lastTranslationValue = translationLimiter.lastValue();
    double lastOmegaLimiterValue = omegaLimiter.lastValue();

    translationLimiter = new SlewRateLimiter(translationLimit);
    omegaLimiter = new SlewRateLimiter(omegaLimit);

    translationLimiter.reset(lastTranslationValue);
    omegaLimiter.reset(lastOmegaLimiterValue);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      flSwerveModule.getPosition(),
      frSwerveModule.getPosition(),
      blSwerveModule.getPosition(),
      brSwerveModule.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      flSwerveModule.getState(),
      frSwerveModule.getState(),
      blSwerveModule.getState(),
      brSwerveModule.getState()
    };
  }

  public double[] getTurnVelocities() {
    return new double[] {
      flSwerveModule.getTurnVelocity(),
      frSwerveModule.getTurnVelocity(),
      blSwerveModule.getTurnVelocity(),
      brSwerveModule.getTurnVelocity()
    };
  }

  public ChassisSpeeds getRobotRelativeSpeeds () {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double getRobotVeloMagnitude() {
    ChassisSpeeds currentSpeeds = kinematics.toChassisSpeeds(getModuleStates());
    double vx = currentSpeeds.vxMetersPerSecond;
    double vy = currentSpeeds.vyMetersPerSecond;

    return Math.hypot(vx, vy);
  }
  
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void zeroOdometry(Rotation2d angle) {
    gyro.resetYaw(0.0);
    flSwerveModule.seedTurnEncoder();
    frSwerveModule.seedTurnEncoder();
    blSwerveModule.seedTurnEncoder();
    brSwerveModule.seedTurnEncoder();
  }
 
  @Override
  public void periodic() {
    if (kS_TunableNumber.hasChanged(hashCode()) || kV_TunableNumber.hasChanged(hashCode())) {
      flSwerveModule.updateFeedforward(new SimpleMotorFeedforward(kS_TunableNumber.get(), kV_TunableNumber.get()));
      frSwerveModule.updateFeedforward(new SimpleMotorFeedforward(kS_TunableNumber.get(), kV_TunableNumber.get()));
      blSwerveModule.updateFeedforward(new SimpleMotorFeedforward(kS_TunableNumber.get(), kV_TunableNumber.get()));
      brSwerveModule.updateFeedforward(new SimpleMotorFeedforward(kS_TunableNumber.get(), kV_TunableNumber.get()));
    }

    if (kP_Drive.hasChanged(hashCode())) {
      flSwerveModule.updateDriveP(kP_Drive.get());
      frSwerveModule.updateDriveP(kP_Drive.get());
      blSwerveModule.updateDriveP(kP_Drive.get());
      brSwerveModule.updateDriveP(kP_Drive.get());
    }

    if (kP_Turn.hasChanged(hashCode())) {
      flSwerveModule.updateTurnP(kP_Turn.get());
      frSwerveModule.updateTurnP(kP_Turn.get() - 0.02);
      blSwerveModule.updateTurnP(kP_Turn.get());
      brSwerveModule.updateTurnP(kP_Turn.get());
    }

    if (DriverStation.isDisabled() || stopped) {
      stop();
      return;
    }

    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(setpoint);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, 
      setpoint, 
      SwerveConstants.MAX_MODULE_SPEED,
      SwerveConstants.MAX_TRANSLATION_SPEED,
      SwerveConstants.MAX_STEER_SPEED
    );

    flSwerveModule.setDesiredState(desiredStates[0]);
    frSwerveModule.setDesiredState(desiredStates[1]);
    blSwerveModule.setDesiredState(desiredStates[2]);
    brSwerveModule.setDesiredState(desiredStates[3]);
  }

  public void stop() { 
    stopped = true;
    flSwerveModule.stop();
    frSwerveModule.stop();
    blSwerveModule.stop();
    brSwerveModule.stop();
  }

  private ChassisSpeeds keepHeadingOutput(ChassisSpeeds speeds) {
    double error = keepHeadingAngle.minus(Rotation2d.fromDegrees(gyro.getYaw())).getDegrees();

    if (Math.abs(speeds.omegaRadiansPerSecond) > 0.01) {
      keepHeadingAngle = Rotation2d.fromDegrees(gyro.getYaw());
    } 
    else if (Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond) > 0.01) {
      speeds.omegaRadiansPerSecond = keepHeadingController.calculate(error); //uses pid 
    }

    return speeds;
  }

  public Command makePath(PoseEstimator poseEstimator) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(poseEstimator.getPoseEstimate().getTranslation(), new Rotation2d()),
      new Pose2d(poseEstimator.getPoseEstimate().getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d())
      //new Pose2d(Units.inchesToMeters(144 - 47), Units.inchesToMeters(158.50 + 10.75), new Rotation2d())
    );
    
    PathConstraints constraint = new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(180));

    PathPlannerPath path = new PathPlannerPath(waypoints, constraint, null, new GoalEndState(0.0, new Rotation2d(Math.PI - 0.139)));
    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public Command pathFindToPose(Pose2d target) {
    PathConstraints constraint = new PathConstraints(1.5, 1.5, Math.PI, Math.PI);

    return AutoBuilder.pathfindToPose(target, constraint, 0.0);
  }

  public void configureAutoBuilder(PoseEstimator poseEstimator) {
    AutoBuilder.configure(
      poseEstimator::getPoseEstimate, 
      poseEstimator::resetPose,
      this::getRobotRelativeSpeeds, 
      this::setDesiredSpeeds, 
      new PPHolonomicDriveController(
        new PIDConstants(1.5, 0, 0),
        new PIDConstants(2.5, 0, 0)
      ), 
      Constants.PATHPLANNER_CONFIG,
        () -> {
          var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, 
        this
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve");
    builder.addDoubleProperty("GyroYaw", this::getYawDegrees, null);
  }
}
