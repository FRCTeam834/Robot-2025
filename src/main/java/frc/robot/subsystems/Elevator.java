// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.TunableNumber;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  //Motors
  private final SparkMax elevatorMotor1;
  private final SparkMax elevatorMotor2;

  //Elevator relative encoder
  private final RelativeEncoder relativeEncoder;

  //PID constants
  private static TunableNumber elevatorkP = new TunableNumber("Elevator/ElevatorkP");
  private static TunableNumber elevatorkI = new TunableNumber("Elevator/ElevatorkI");
  private static TunableNumber elevatorkD = new TunableNumber("Elevator/ElevatorkD");

  //Feedforward constants
  private static TunableNumber elevatorkS = new TunableNumber("Elevator/ElevatorkS"); //static friction
  private static TunableNumber elevatorkG = new TunableNumber("Elevator/ElevatorkG"); //gravity
  private static TunableNumber elevatorkV = new TunableNumber("Elevator/ElevatorkV"); //velocity
  private static TunableNumber elevatorkA = new TunableNumber("Elevator/ElevatorkA"); //acceleration

  //Initialize feedforward
  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);

  //PID controller
  private SparkClosedLoopController pid_controller;

  //Motor configurations
  private SparkMaxConfig motor1Config = new SparkMaxConfig();
  private SparkMaxConfig motor2Config = new SparkMaxConfig();

  private double setpointHeight = 0.0; //m
  private boolean elevatorStopped = true;

  static {
    //TODO: Intitialize default constants
    elevatorkP.initDefault(5);
    elevatorkI.initDefault(0);
    elevatorkD.initDefault(0);

    elevatorkS.initDefault(0.5); 
    elevatorkG.initDefault(0.8);
    elevatorkV.initDefault(0.0); // Do not use with MAXMotion
    elevatorkA.initDefault(0.0);
  }

  //Elevator constructor
  public Elevator() {
    //Initialize motors, encoders, PID controller
    elevatorMotor1 = new SparkMax(10, MotorType.kBrushless);
    elevatorMotor2 = new SparkMax(11, MotorType.kBrushless);
    relativeEncoder = elevatorMotor1.getEncoder();
    
    pid_controller = elevatorMotor1.getClosedLoopController();

    //Configure motors
    motor1Config
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .voltageCompensation(12)
    //.closedLoopRampRate(0.2)
    .inverted(true);

    //Configure motor encoders
    motor1Config.encoder
    .positionConversionFactor((Math.PI * Units.inchesToMeters(1.5)) / 5) // Add gearing
    .velocityConversionFactor(((Math.PI * Units.inchesToMeters(1.5)) / 5) / 60);

    //Configure PID controller
    motor1Config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(elevatorkP.get(), elevatorkI.get(), elevatorkD.get())
    .outputRange(-1, 1)
    .maxMotion
    .maxAcceleration(1)
    .maxVelocity(0.5)
    .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
    .allowedClosedLoopError(0.01);

    //Second motor should have the same configurations as the first motor
    motor2Config.apply(motor1Config);
    motor2Config.follow(elevatorMotor1, true);

    elevatorMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    relativeEncoder.setPosition(0.0);

    //Send all data to the Smart Dashboard
    SmartDashboard.putData(this);
  }

  //Called once per scheduler run
  @Override
  public void periodic() {
    
    //Update the elevator PID if the constants have changed
    if (elevatorkP.hasChanged(hashCode()) || elevatorkI.hasChanged(hashCode()) || elevatorkD.hasChanged(hashCode())) {
      motor1Config.closedLoop.pid(elevatorkP.get(), elevatorkI.get(), elevatorkD.get());
      motor2Config.closedLoop.pid(elevatorkP.get(), elevatorkI.get(), elevatorkD.get());
      elevatorMotor1.configureAsync(motor1Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      elevatorMotor2.configureAsync(motor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    //Update the elevator feedforward if the constants have changed
    if (elevatorkS.hasChanged(hashCode()) || elevatorkG.hasChanged(hashCode()) || elevatorkV.hasChanged(hashCode()) || elevatorkA.hasChanged(hashCode())) {
      elevatorFeedforward = new ElevatorFeedforward(elevatorkS.get(), elevatorkG.get(), elevatorkV.get(), elevatorkA.get());
    }

    //Update PID controller
    if (!elevatorStopped) {
      pid_controller.setReference(setpointHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, elevatorFeedforward.calculate(relativeEncoder.getVelocity()));
    }
  }

  public void zeroEncoder() {
    relativeEncoder.setPosition(0.0);
  }

  public double getElevatorHeight() {
    return relativeEncoder.getPosition();
  }

  public void setElevatorSpeed(double speed) {
    elevatorMotor1.set(speed);
  }

  //Stop the elevator motors
  public void stop() {
    elevatorStopped = true;
    elevatorMotor1.setVoltage(0.0); //Stop motors
  }

  //Update the setpoint for the elevator
  public void setDesiredPosition(double height) {
    elevatorStopped = false;
    this.setpointHeight = height;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getElevatorHeight() - setpointHeight) < Units.inchesToMeters(1);
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");
  }
}
