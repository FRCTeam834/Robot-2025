// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import frc.robot.Constants.GamePiece;
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
  private final RelativeEncoder intakeEncoder;

  private LaserCan laserCAN;

  public GamePiece currentPiece = GamePiece.NONE;

  //Arm PID constants
  private static final TunableNumber pivotkP = new TunableNumber("Arm/armkP");
  private static final TunableNumber pivotkI = new TunableNumber("Arm/armkI");
  private static final TunableNumber pivotkD = new TunableNumber("Arm/armkD");
  
  //Arm feedforward constants
  private static final TunableNumber pivotkS = new TunableNumber("Arm/armkS"); //static friction
  private static final TunableNumber pivotkG = new TunableNumber("Arm/armkG"); //gravity
  private static final TunableNumber pivotkV = new TunableNumber("Arm/armkV"); //velocity
  private static final TunableNumber pivotkA = new TunableNumber("Arm/armkA");

  //Initialize arm feedforward
  private ArmFeedforward armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
  
  //Arm PID controller
  private ProfiledPIDController trapezoidPID = new ProfiledPIDController(0, 0, 0, 
    new TrapezoidProfile.Constraints(Units.degreesToRadians(45), Units.degreesToRadians(45))
    //new TrapezoidProfile.Constraints(Units.degreesToRadians(70), Units.degreesToRadians(70))
  );

  //Booleans for if the arm is stopped
  private boolean armStopped = true;
  
  //Arm angle setpoint
  private double pivotAngleSetpoint = 0.0; 

  //Intake voltag  
  //Configuration for the motors
  private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
 
  //Arm PID and feedforward constants
  static {
    //TODO: Initialize default constants
    pivotkP.initDefault(1);
    pivotkI.initDefault(0);
    pivotkD.initDefault(0);

    pivotkS.initDefault(0.66);
    pivotkG.initDefault(0.35); //0.55
    pivotkV.initDefault(2.3); // 1.6
    pivotkA.initDefault(0.04);
  }

  //Arm constructor
  public Arm() {
    //Initialize motor
    pivotMotor = new SparkMax(ArmConstants.pivotCANID, MotorType.kBrushless);
    intakeMotor = new SparkMax(ArmConstants.intakeCANID, MotorType.kBrushless);
    laserCAN = new LaserCan(ArmConstants.laserCANID);
    
    //Initialize arm absolute encoder
    pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
    intakeEncoder = intakeMotor.getEncoder();
    
    //Configure motors and encoders
    pivotMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12)
    .inverted(true);

    pivotMotorConfig.absoluteEncoder
    .positionConversionFactor(2.0 * Math.PI)
    .velocityConversionFactor(2.0 * Math.PI / 60.0)
    .inverted(true);

    intakeMotorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12)
    .inverted(false);

    intakeMotorConfig.encoder
    .positionConversionFactor(2.0 * Math.PI)
    .velocityConversionFactor(2.0 * Math.PI / 60.0);

    //Apply configurations
    pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeEncoder.setPosition(0);

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
    if (pivotkS.hasChanged(hashCode()) || pivotkG.hasChanged(hashCode()) || pivotkV.hasChanged(hashCode()) || pivotkA.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(pivotkS.get(), pivotkG.get(), pivotkV.get(), pivotkA.get());
    }

    //Set the position of the arm motor
    if (!armStopped) {
      pivotMotor.setVoltage(trapezoidPID.calculate(getCurrentPivotAngle()) + 
      armFeedforward.calculate(trapezoidPID.getSetpoint().position + (Math.PI/2), trapezoidPID.getSetpoint().velocity));
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
    pivotAngleSetpoint = MathUtil.clamp(angle, ArmConstants.MAXIMUM_ANGLE, 0.2);
    trapezoidPID.reset(getCurrentPivotAngle());
    trapezoidPID.setGoal(new TrapezoidProfile.State(pivotAngleSetpoint, 0.0));
  }

  public void setIntakeVoltage (double volts) {
    intakeMotor.setVoltage(volts);
  }

  public double getIntakeOutputCurrent () {
    return intakeMotor.getAppliedOutput();
  }

  public double getIntakeAngle() {
    return intakeEncoder.getPosition();
  }

  public void setPivotVoltage(double volts) {
    pivotMotor.setVoltage(volts);
  }

  public void stowArm() {
    setDesiredPivotAngle(0.0);
  }

  public void homeArm() {
    setDesiredPivotAngle(0.3);
  }

  public double getCurrentPivotAngle() {
    return MathUtil.angleModulus(pivotAbsoluteEncoder.getPosition());
  }

  public boolean atSetpointAngle () {
    return Math.abs(getCurrentPivotAngle() - pivotAngleSetpoint) < ArmConstants.ANGLE_TOLERANCE;
  }

  public double getLaserCANMeasurement() {
    LaserCan.Measurement measurement = laserCAN.getMeasurement();
    if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) return 999;
    return measurement.distance_mm;
  }

  public boolean hasCoral () {
    return getLaserCANMeasurement() < 30.0;
  }

  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("CurrentAngle", this::getCurrentPivotAngle, null);
    builder.addDoubleProperty("SetpointAngle", () -> {return this.pivotAngleSetpoint;}, null);
    builder.addDoubleProperty("laserCAN_distance", this::getLaserCANMeasurement, null);

    builder.addDoubleProperty("PIDSetpoint_Position", () -> { return trapezoidPID.getSetpoint().position; }, null);
    builder.addDoubleProperty("PIDSetpoint_Velocity", () -> { return trapezoidPID.getSetpoint().velocity; }, null);

    builder.addDoubleProperty("PIDVoltage", () -> { return trapezoidPID.calculate(getCurrentPivotAngle()); }, null);
    builder.addDoubleProperty("FF Voltage", () -> { return armFeedforward.calculate(trapezoidPID.getSetpoint().position + (Math.PI/2), trapezoidPID.getSetpoint().velocity); }, null);
  }
}
