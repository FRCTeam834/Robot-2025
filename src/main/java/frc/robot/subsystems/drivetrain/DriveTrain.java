// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

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

  private SwerveDriveOdometry odometry;

  private boolean stopped = false;
  private ChassisSpeeds setpoint = new ChassisSpeeds();

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
    SmartDashboard.putData(this);

    odometry = new SwerveDriveOdometry( 
      kinematics, 
      getYaw(),
      getModulePositions()
    );
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
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getYaw());
    setDesiredSpeeds(speeds);
  }

  public void setDesiredSpeeds(ChassisSpeeds speeds) {
    stopped = false;
    setpoint = speeds;
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
  
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void zeroOdometry(Rotation2d angle) {
    gyro.resetYaw(0.0);
    odometry.resetRotation(angle);
    flSwerveModule.resetDriveEncoder();
    frSwerveModule.resetDriveEncoder();
    blSwerveModule.resetDriveEncoder();
    brSwerveModule.resetDriveEncoder();
  }
 
  @Override
  public void periodic() {
    odometry.update(getYaw(), getModulePositions());
    
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve");
    builder.addDoubleProperty("GyroYaw", this::getYawDegrees, null);
  }
}
