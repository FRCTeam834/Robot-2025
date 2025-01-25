// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//! Errors importing revrobotics
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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.TunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;



public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  //Booleans for if the arm is stopped
  private boolean armStopped = false;
  private boolean intakeStopped = true; //!Intake

  //Arm angle setpoint
  private double angleSetpoint = 0.0; //!Radians? Degrees? Rotations?

  //Motors
  private final SparkMax intakeMotor; //! Intake
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

  //Configuration for the arm motor
  private SparkMaxConfig armMotorConfig = new SparkMaxConfig();
 
  //Arm PID and feedforward constants
  static {
    //!Update with the correct default PID and feedforward constants
    armkP.initDefault(1);
    armkI.initDefault(0);
    armkD.initDefault(0);

    armkS.initDefault(0);
    armkG.initDefault(0);
    armkV.initDefault(0);
  }

  //Arm constructor
  public Arm() {
    //Initialize arm motor
    armMotor = new SparkMax(17, MotorType.kBrushless); //!Update with correct motor ID
    
    //Initialize arm absolute encoder
    armAbsoluteEncoder = armMotor.getAbsoluteEncoder();

    //Initialize arm PID controller
    armPIDController = armMotor.getClosedLoopController();
    
    //Configure arm motor
    //TODO: Arm motor configurations

    //TODO: Arm Encoder configurations

    //TODO: Arm PID configurations
    
    armMotor.configure(armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Send all data to the Smart Dashboard
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (armkP.hasChanged(hashCode()) || armkI.hasChanged(hashCode()) || armkD.hasChanged(hashCode())) {
      SparkMaxConfig updateConfig = new SparkMaxConfig();
      updateConfig.closedLoop.pid(armkP.get(), armkI.get(), armkD.get());
    }

    if (armkS.hasChanged(hashCode()) || armkG.hasChanged(hashCode()) || armkV.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(armkS.get(), armkG.get(), armkV.get());
    }


    //!Incomplete
    if (!armStopped) {
      
    }
  }

  //!Incomplete
  public void stop () {
    armStopped = true;
    intakeStopped = true;
    armMotor.setVoltage(0.0);
  }

  //!Incomplete
  public void setDesiredArmAngle (double angle) {
  }

  public void setIntakeVoltage (double volts) {
    
  }

  public void initSendable (SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
  }
}
