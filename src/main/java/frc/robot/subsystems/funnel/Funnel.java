// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.funnel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;

public class Funnel extends SubsystemBase {
  private final SparkMax funnelMotor;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();

  private final RelativeEncoder motorEncoder;
  private final PIDController funnelPID = new PIDController(1, 0, 0);
  
  private boolean isMotorRunning = false;
  private double setpointAngle = 0.0;

  public Funnel() {
    funnelMotor = new SparkMax(FunnelConstants.funnelMotorID, MotorType.kBrushless);
    motorEncoder = funnelMotor.getEncoder();

    motorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12)
    .inverted(false);

    motorConfig.encoder
    .positionConversionFactor(2 * Math.PI)
    .velocityConversionFactor(2 * Math.PI / 60);

    funnelMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorEncoder.setPosition(0.0);
    funnelPID.setTolerance(Units.degreesToRadians(2));
  }

  public void zeroEncoder() {
    motorEncoder.setPosition(0.0);
  }

  public double getCurrentRotorAngle() {
    return motorEncoder.getPosition();
  }

  public void setDesiredAngle(double angle) {
    isMotorRunning = true;
    setpointAngle = angle;
    funnelPID.setSetpoint(setpointAngle);
  }

  public void setVoltage(double voltage) {
    funnelMotor.setVoltage(voltage);
  }

  public void stop() {
    isMotorRunning = false;
    funnelMotor.setVoltage(0.0);
  }

  public boolean isAtSetpoint() {
    return funnelPID.atSetpoint();
  }


  @Override
  public void periodic() {
    if (isMotorRunning) {
      funnelMotor.setVoltage(funnelPID.calculate(setpointAngle));
    }
  }
}
