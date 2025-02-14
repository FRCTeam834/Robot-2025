// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.util.sendable.SendableBuilder;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  //Motor
  private final SparkMax climberMotor;

  //Relative encoder
  private final RelativeEncoder climberRelativeEncoder;

  //Configuration
  private SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
  
  //Variables
  //TODO: Questions
  /**
   * Is a PID needed?
   * Is a feedforward needed?
   * 
   */
  public Climber() {
    //Initialize motor and encoder
    climberMotor = new SparkMax(19, MotorType.kBrushless);
    climberRelativeEncoder = climberMotor.getEncoder();

    //TODO: Correct motor configurations
    
    //Motor configuration
    climberMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .voltageCompensation(12)
    .inverted(false);

    //Encoder configurations
    climberMotorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

    //TODO: Correct reset and persist parameters?
    climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    //Set relative encoder position to 0
    climberRelativeEncoder.setPosition(0);

    //Send all data to the Smart Dashboard
    //TODO: Is a smartdashboard needed for the climber?
    //SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClimberMotorVoltage(double voltage) {
    climberMotor.setVoltage(voltage);
  }

  public void stop() {
    climberMotor.setVoltage(0.0);
  }

  //TODO: Is initSendable needed for climber? What does it do?
  /*
  @Override
  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("Climber");
  }

  */
}
