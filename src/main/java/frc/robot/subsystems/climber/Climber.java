// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final SparkMax climberMotor;

  private SparkMaxConfig climberMotorConfig = new SparkMaxConfig();

  public Climber() {
    climberMotor = new SparkMax(ClimberConstants.climberMotorID, MotorType.kBrushless);

    climberMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .voltageCompensation(12)
    .inverted(true);

    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimberVoltage(double voltage) {
    climberMotor.setVoltage(voltage);
  }

  public void stop() {
    climberMotor.setVoltage(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
