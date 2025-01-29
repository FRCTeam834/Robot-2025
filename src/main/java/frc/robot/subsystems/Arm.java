// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.TunableNumber;
////import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  //Declare motors
  private final SparkMax intakeMotor;
  private final SparkMax armMotor;
  
  //Arm absolute encoder
  private final AbsoluteEncoder armAbsoluteEncoder;

  //Arm PID constants
  private static final TunableNumber armkP = new TunableNumber("Arm/armkP");
  private static final TunableNumber armkI = new TunableNumber("Arm/armkI");
  private static final TunableNumber armkD = new TunableNumber("Arm/armkD");
  
  //Arm feedforward constants
  private static final TunableNumber armkS = new TunableNumber("Arm/armkS"); //static friction
  private static final TunableNumber armkG = new TunableNumber("Arm/armkG"); //gravity
  private static final TunableNumber armkV = new TunableNumber("Arm/armkV"); //velocity

  //Initialize arm feedforward
  private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  
  //Arm PID controller
  private SparkClosedLoopController armPIDController;

  //Booleans for if the arm is stopped
  private boolean armStopped = false;
  private boolean intakeStopped = false;
  
  //Arm angle setpoint
  private double angleSetpoint = 0.0; //TODO: Get units

  //Intake voltage
  private double intakeVoltage = 0.0; //TODO: Find correct values for intake voltages
  
  //Configuration for the motors
  private SparkMaxConfig armMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
 
  //Arm PID and feedforward constants
  static {
    //TODO: Initialize default constants
    armkP.initDefault(1);
    armkI.initDefault(0);
    armkD.initDefault(0);

    armkS.initDefault(0);
    armkG.initDefault(0);
    armkV.initDefault(0);
  }

  //Arm constructor
  public Arm() {
    //Initialize motor
    armMotor = new SparkMax(17, MotorType.kBrushless);
    intakeMotor = new SparkMax(18, MotorType.kBrushless);
    
    //Initialize arm absolute encoder
    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();

    //Initialize arm PID controller
    armPIDController = armMotor.getClosedLoopController();
    
    //Configure motors and encoders
    //TODO: Arm motor configurations
    armMotorConfig
    .idleMode(IdleMode.kBrake)
    .inverted(false);

    //TODO: Arm Encoder configurations
    armMotorConfig.encoder
    .positionConversionFactor(0)
    .velocityConversionFactor(0);

    //TODO: Arm PID configurations
    armMotorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(armkP.get(), armkI.get(), armkD.get());

    //TODO: Intake motor configurations
    intakeMotorConfig
    .idleMode(IdleMode.kBrake)
    .inverted(false);

    //Apply configurations
    armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Send all data to the Smart Dashboard
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //Update the arm PID if the constants have changed
    if (armkP.hasChanged(hashCode()) || armkI.hasChanged(hashCode()) || armkD.hasChanged(hashCode())) {
      armMotorConfig.closedLoop.pid(armkP.get(), armkI.get(), armkD.get());
      armMotor.configureAsync(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    //Update the arm feedforward if the constants have changed
    if (armkS.hasChanged(hashCode()) || armkG.hasChanged(hashCode()) || armkV.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(armkS.get(), armkG.get(), armkV.get());
    }

    //Set the position of the arm motor
    if (!armStopped) {
      armPIDController.setReference(angleSetpoint, ControlType.kPosition, null, armFeedforward.calculate(armAbsoluteEncoder.getPosition(), armAbsoluteEncoder.getVelocity()));
    }

    //Set the voltage of the intake motor
    if (!intakeStopped) {
      intakeMotor.setVoltage(intakeVoltage);
    }
  }

  //Stop all motors
  public void stop () {
    armStopped = true;
    intakeStopped = true;
    armMotor.setVoltage(0.0);
    intakeMotor.setVoltage(0.0);
  }

  //Update the arm angle setpoint
  public void setArmAngleSetpoint (double angle) {
    armStopped = false;
    angleSetpoint = angle;
  }

  //Update the desired intake voltage
  public void setIntakeVoltage (double volts) {
    intakeStopped = false;
    intakeVoltage = volts;
  }

  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
  }
}
