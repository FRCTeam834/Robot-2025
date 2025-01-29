// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.CAN;
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
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.2, SwerveConstants.drivingVelocityFeedForward);

  public SwerveModule(int steerID, int driveID, int CANCoderID, double encoderOffset, boolean reversedDrive) {
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    turnMotor = new SparkMax(steerID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
    turnCANCoder = new CANcoder(CANCoderID);

    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    driveMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12)
    .inverted(reversedDrive);
    //.closedLoopRampRate(0.5);

    driveMotorConfig.encoder
    .positionConversionFactor((Math.PI * SwerveConstants.MODULE_WHEEL_DIAMETER) / SwerveConstants.DRIVE_GEAR_RATIO) // meters 
    .velocityConversionFactor((Math.PI * SwerveConstants.MODULE_WHEEL_DIAMETER) / (60.0 * SwerveConstants.DRIVE_GEAR_RATIO)); // meters per second

    driveMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.15, 0, 0)
    .velocityFF(0) //SwerveConstants.drivingVelocityFeedForward
    .outputRange(-1, 1)
    .maxMotion //TODO
      .maxAcceleration(1.0)
      .maxVelocity(1.0)
      .allowedClosedLoopError(.1);

    turnMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(20)
    .voltageCompensation(12)
    .inverted(false);

    turnMotorConfig.encoder
    .positionConversionFactor(2 * Math.PI / SwerveConstants.STEER_GEAR_RATIO)
    .velocityConversionFactor(2 * Math.PI / (60 * SwerveConstants.STEER_GEAR_RATIO));

    turnMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(0.25, 0, 0.0)
    .outputRange(-1, 1)
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(-Math.PI, Math.PI);

    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setpoint.angle = Rotation2d.fromRadians(getCANCoderAngle()); 
    turnEncoder.setPosition(getCANCoderAngle());

    SmartDashboard.putData("SwerveModule " + driveID, this);
  }

  public double getCANCoderAngle() {
    return Units.rotationsToRadians(turnCANCoder.getAbsolutePosition().getValueAsDouble()); 
  }

  public double getRawCANCoderAngle() {
    return turnCANCoder.getAbsolutePosition().getValueAsDouble();
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

  public double getSetpointSpeed() {
    return setpoint.speedMetersPerSecond;
  }

  public double getSetpointAngle() {
    return setpoint.angle.getRadians();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromRadians(getCANCoderAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromRadians(getCANCoderAngle()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle;

    correctedDesiredState.optimize(Rotation2d.fromRadians(getCANCoderAngle()));

    if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    // correctedDesiredState.speedMetersPerSecond *= Math.cos(correctedDesiredState.speedMetersPerSecond - getTurnAngle());

    //TODO: MAXMotion
    //TODO: If needed try arbitary feedforward
    driveController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff.calculate(correctedDesiredState.speedMetersPerSecond));
    turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.setpoint = correctedDesiredState;
  }

  public void seedTurnEncoder() {
    turnEncoder.setPosition(getCANCoderAngle());
  }

  public void checkAndSeedTurnEncoder() {
    if(Math.abs(getCANCoderAngle() - getTurnAngle()) > Units.degreesToRadians(5) && getTurnVelocity() < 0.005) {
      seedTurnEncoder();
    }
  }

  public void zeroCANCoder() {
    //wheels currently in desired zero position
    CANcoderConfiguration getConfig = new CANcoderConfiguration();
    turnCANCoder.getConfigurator().refresh(getConfig);
    double oldOffset = getConfig.MagnetSensor.MagnetOffset;

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = -(turnCANCoder.getAbsolutePosition().getValueAsDouble() - oldOffset);
    turnCANCoder.getConfigurator().apply(config);
    seedTurnEncoder();
  }

  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }

  public void stop() {
    driveMotor.setVoltage(0);
    turnMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    //checkAndSeedTurnEncoder();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");

    builder.addDoubleProperty("AbsoluteEncoderAngle", this::getCANCoderAngle, null);
    builder.addDoubleProperty("Setpoint Speed", this::getSetpointSpeed, null);
    builder.addDoubleProperty("Setpoint Angle", this::getSetpointAngle, null);
    builder.addDoubleProperty("Speed", this::getDriveVelocity, null);
    builder.addDoubleProperty("Angle", this::getTurnAngle, null);
  }
}
