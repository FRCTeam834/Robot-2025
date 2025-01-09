// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class SwerveModule extends SubsystemBase {
  /** Creates a new Swerve. */  

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final CANcoder turnCanCoder;

  private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  private final double DRIVE_GEAR_RATIO = 50.0/14.0;
  private final double WHEEL_CIRCUMFERENCE_METER = Math.PI*6.0/1000.0;

  public SwerveModule(int steerID, int driveID, int CANCoderID) {
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    turnMotor = new SparkMax(driveID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnCanCoder = new CANcoder(CANCoderID);

    turnCanCoder.setPosition(0);

    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();

    driveMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .inverted(false)
    .openLoopRampRate(0.5);

    driveMotorConfig.encoder
    .positionConversionFactor(1.0/DRIVE_GEAR_RATIO*WHEEL_CIRCUMFERENCE_METER)
    .velocityConversionFactor(1.0/DRIVE_GEAR_RATIO*WHEEL_CIRCUMFERENCE_METER/60.0);

    turnMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(30)
    .inverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
