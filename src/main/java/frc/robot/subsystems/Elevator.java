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
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.TunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  //Motors
  private final SparkMax elevatorMotor1;
  private final SparkMax elevatorMotor2;

  //Elevator absolute encoder
  private final AbsoluteEncoder absoluteEncoder;

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

  private SparkMaxConfig motor1Config = new SparkMaxConfig();
  private SparkMaxConfig motor2Config = new SparkMaxConfig();

  private double setpoint = 0.0; //m
  private boolean elevatorStopped = false;

  static {
    //TODO: Intitialize default constants
    elevatorkP.initDefault(1);
    elevatorkI.initDefault(0);
    elevatorkD.initDefault(0);

    elevatorkS.initDefault(0);
    elevatorkG.initDefault(0);
    elevatorkV.initDefault(0);
    elevatorkA.initDefault(0);
  }

  //Elevator constructor
  public Elevator() {
    //Initialize and configure motors
    elevatorMotor1 = new SparkMax(15, MotorType.kBrushless);
    elevatorMotor2 = new SparkMax(16, MotorType.kBrushless);
    absoluteEncoder = elevatorMotor1.getAbsoluteEncoder();

    pid_controller = elevatorMotor1.getClosedLoopController();

    //Configure motors
    motor1Config
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(50)
    .voltageCompensation(12)
    .closedLoopRampRate(0.2)
    .inverted(false);

    //Configure motor encoders
    motor1Config.encoder
    .positionConversionFactor(Math.PI) //TODO: get meters
    .velocityConversionFactor(Math.PI / 60); //TODO: get meters/sec

    //Configure PID controller
    motor1Config.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(elevatorkP.get(), elevatorkI.get(), elevatorkD.get())
    .outputRange(-1, 1)
    .maxMotion
    .maxAcceleration(1.0)
    .maxVelocity(1.0)
    .allowedClosedLoopError(0.1);

    //Second motor should have the same configurations as the first motor
    motor2Config.apply(motor1Config);
    motor2Config.follow(elevatorMotor1);

    elevatorMotor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Send all data to the Smart Dashboard
    SmartDashboard.putData(this);
  }

  //Called once per scheduler run
  @Override
  public void periodic() {
    
    //Update the elevator PID if the constants have changed
    if (elevatorkP.hasChanged(hashCode()) || elevatorkI.hasChanged(hashCode()) || elevatorkD.hasChanged(hashCode())) {
      SparkMaxConfig updateConfig = new SparkMaxConfig();
      updateConfig.closedLoop.pid(elevatorkP.get(), elevatorkI.get(), elevatorkD.get());
      elevatorMotor1.configureAsync(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    //Update the elevator feedforward if the constants have changed
    if (elevatorkS.hasChanged(hashCode()) || elevatorkG.hasChanged(hashCode()) || elevatorkV.hasChanged(hashCode()) || elevatorkA.hasChanged(hashCode())) {
      elevatorFeedforward = new ElevatorFeedforward(elevatorkS.get(), elevatorkG.get(), elevatorkV.get(), elevatorkA.get());
    }

    //Update PID controller
    if (!elevatorStopped) {
      pid_controller.setReference(setpoint, ControlType.kPosition, null, elevatorFeedforward.calculate(absoluteEncoder.getVelocity())); //!Is this the correct parameter?
    }
  }

  //Stop the elevator motors
  public void stop() {
    elevatorStopped = true;
    elevatorMotor1.setVoltage(0.0); //Stop motors
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");
  }
}
