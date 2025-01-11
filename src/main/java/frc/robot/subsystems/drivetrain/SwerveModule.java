// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
public class SwerveModule extends SubsystemBase {
  /** Creates a new Swerve. */  

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final CANcoder turnCANCoder;

  private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private SwerveModuleState setpoint = new SwerveModuleState();

  public SwerveModule(int steerID, int driveID, int CANCoderID) {
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    turnMotor = new SparkMax(driveID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    turnCANCoder = new CANcoder(CANCoderID);
    // ? Seeding or direct cancoder value

    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    driveMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .voltageCompensation(12)
    .inverted(false);
    //.closedLoopRampRate(0.5);

    driveMotorConfig.encoder
    .positionConversionFactor(SwerveConstants.drivingFactor) // meters 
    .velocityConversionFactor(SwerveConstants.drivingFactor / 60.0); // meters per second

    driveMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.04, 0, 0)
    .velocityFF(SwerveConstants.drivingVelocityFeedForward)
    .outputRange(-1, 1)
    .maxMotion
      .maxAcceleration(1.0)
      .maxVelocity(1.0)
      .allowedClosedLoopError(.1);

    turnMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(30)
    .voltageCompensation(12)
    .inverted(false);

    turnMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1, 0, 0)
    .outputRange(-1, 1)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(0, SwerveConstants.turningFactor);

    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setpoint.angle = Rotation2d.fromRotations(getCANCoderAngle()); 
    turnEncoder.setPosition(getCANCoderAngle());
    driveEncoder.setPosition(0);
    turnCANCoder.setPosition(0);
    
    SmartDashboard.putData("SwerveModule " + driveID, this);
  }

  public double getCANCoderAngle() {
    return turnCANCoder.getAbsolutePosition().getValueAsDouble(); // ! rotations
  }

  public double getTurnAngle() {
    return turnEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurnVelocity() {
    return turnEncoder.getVelocity();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRotations(getCANCoderAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRotations(getCANCoderAngle()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle;

    correctedDesiredState.optimize(Rotation2d.fromRotations(getCANCoderAngle()));

    if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    // correctedDesiredState.speedMetersPerSecond *= Math.cos(correctedDesiredState.speedMetersPerSecond - getTurnAngle());

    driveController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.setpoint = correctedDesiredState;
  }

  public void seedTurnEncoder() {
    turnEncoder.setPosition(getCANCoderAngle());
  }

  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }

  public void stop() {
    driveMotor.setVoltage(0);
    turnMotor.setVoltage(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");

    builder.addDoubleProperty("Setpoint Speed", this::getSetpointSpeed, null);
    builder.addDoubleProperty("Setpoint Angle", this::getSetpointAngle, null);
    builder.addDoubleProperty("Raw Setpoint Angle", () -> { return rawSetpoint.angle.getRadians(); }, null);
    builder.addDoubleProperty("Speed", this::getSpeed, null);
    builder.addDoubleProperty("Angle", this::getAngleRadians, null);
    builder.addDoubleProperty("PID Voltage", () -> driveController.calculate(inputs.driveVelocity, setpoint.speedMetersPerSecond), null);
  }
}
