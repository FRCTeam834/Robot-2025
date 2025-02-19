// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.utility.TunableNumber;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  //Declare motors
  private final SparkMax intakeMotor;
  private final SparkMax pivotMotor;
  
  //Arm absolute encoder
  private final AbsoluteEncoder pivotAbsoluteEncoder;

  private LaserCan laserCAN;

  //Arm PID constants
  private static final TunableNumber pivotkP = new TunableNumber("Arm/armkP");
  private static final TunableNumber pivotkI = new TunableNumber("Arm/armkI");
  private static final TunableNumber pivotkD = new TunableNumber("Arm/armkD");
  
  //Arm feedforward constants
  private static final TunableNumber pivotkS = new TunableNumber("Arm/armkS"); //static friction
  private static final TunableNumber pivotkG = new TunableNumber("Arm/armkG"); //gravity
  private static final TunableNumber pivotkV = new TunableNumber("Arm/armkV"); //velocity

  //Initialize arm feedforward
  private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  
  //Arm PID controller
  private ProfiledPIDController trapezoidPID = new ProfiledPIDController(0, 0, 0, 
    new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180))
  );

  //Booleans for if the arm is stopped
  private boolean armStopped = true;
  
  //Arm angle setpoint
  private double pivotAngleSetpoint = 0.0; 

  //Intake voltag  
  //Configuration for the motors
  private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
  private AbsoluteEncoderConfig pivotEncoderConfig = new AbsoluteEncoderConfig();
 
  //Arm PID and feedforward constants
  static {
    //TODO: Initialize default constants
    pivotkP.initDefault(1);
    pivotkI.initDefault(0);
    pivotkD.initDefault(0);

    pivotkS.initDefault(0);
    pivotkG.initDefault(0);
    pivotkV.initDefault(0);
  }

  //Arm constructor
  public Arm() {
    //Initialize motor
    pivotMotor = new SparkMax(ArmConstants.pivotCANID, MotorType.kBrushless);
    intakeMotor = new SparkMax(ArmConstants.intakeCANID, MotorType.kBrushless);
    laserCAN = new LaserCan(ArmConstants.laserCANID);
    
    //Initialize arm absolute encoder
    pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
    
    //Configure motors and encoders
    pivotMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12)
    .inverted(false);

    intakeMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12)
    .inverted(false);

    pivotEncoderConfig
    .positionConversionFactor(2 * Math.PI)
    .velocityConversionFactor(2 * Math.PI / 60)
    .zeroOffset(0)
    .setSparkMaxDataPortConfig(); //revolutions

    //Apply configurations
    pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //Update the arm PID if the constants have changed
    if (pivotkP.hasChanged(hashCode()) || pivotkI.hasChanged(hashCode()) || pivotkD.hasChanged(hashCode())) {
      trapezoidPID.setPID(pivotkP.get(), pivotkI.get(), pivotkD.get());
    }

    //Update the arm feedforward if the constants have changed
    if (pivotkS.hasChanged(hashCode()) || pivotkG.hasChanged(hashCode()) || pivotkV.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(pivotkS.get(), pivotkG.get(), pivotkV.get());
    }

    //Set the position of the arm motor
    if (!armStopped) {
      pivotMotor.setVoltage(trapezoidPID.calculate(pivotAngleSetpoint) + 
      armFeedforward.calculate(trapezoidPID.getSetpoint().position, trapezoidPID.getSetpoint().velocity));
    }
  }

  //Stop all motors
  public void stop () {
    armStopped = true;
    pivotMotor.setVoltage(0.0);
    intakeMotor.setVoltage(0.0);
  }

  //Update the arm angle setpoint
  public void setDesiredPivotAngle (double angle) {
    armStopped = false;
    pivotAngleSetpoint = MathUtil.clamp(angle, 0, Math.PI);
  }

  public void setIntakeVoltage (double volts) {
    intakeMotor.setVoltage(volts);
  }

  public double getCurrentPivotAngle() {
    return pivotAbsoluteEncoder.getPosition();
  }

  public double getLaserCANMeasurement() {
    LaserCan.Measurement measurement = laserCAN.getMeasurement();
    if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return -1.0;
    return laserCAN.getMeasurement().distance_mm;
  }

  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("CurrentAngle", this::getCurrentPivotAngle, null);
    builder.addDoubleProperty("SetpointAngle", () -> {return this.pivotAngleSetpoint;}, null);
    builder.addDoubleProperty("laserCAN_distance", this::getLaserCANMeasurement, null);
  }
}
