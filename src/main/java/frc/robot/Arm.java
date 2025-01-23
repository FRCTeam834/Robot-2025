// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.utility.TunableNumber;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  //Booleans for if the arm is stopped
  private boolean armStopped = true;
  private boolean intakeStopped = true;
  private double desiredAngle = 0.0;
  private static double currentArmAngle = 0;
  private double armAppliedVoltage = 0.0;

  //Motors
  private final CANSparkMax intakeMotor;

  //Arm PID constants
  private static final TunableNumber armkP = new TunableNumber("Arm/armkP");
  private static final TunableNumber armkI = new TunableNumber("Arm/armkI");
  private static final TunableNumber armkD = new TunableNumber("Arm/armkD");

  //Arm feedforward constants
  private static final TunableNumber armkS = new TunableNumber("Arm/armkS");
  private static final TunableNumber armkG = new TunableNumber("Arm/armkG");
  private static final TunableNumber armkV = new TunableNumber("Arm/armkV");

  //Feedforward and PID for the arm
  private ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);
  //!Update with the correct constraints
  private final ProfiledPIDController armPID = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.degreesToRadians(/*Contraint*/), Units.degreesToRadians(/*Constraint*/)));
  
  //Arm PID and feedforward constants
  static {
    //!Update with the correct default PID and feedforward constants
    armkP.initDefault(1);
    armkD.initDefault(0);
    armkI.initDefault(0);
    armkS.initDefault(0);
    armkG.initDefault(0);
    armkV.initDefault(0);
  }

  public Arm() {
    intakeMotor = new CANSparkMax(3, MotorType.kBrushless); //!Update with correct motor ID
    
    pivotPID.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (armkP.hasChanged(hashCode()) || armkI.hasChanged(hashCode()) || armkD.hasChanged(hashCode())) {
      armPID.setPID(pivotkP.get(), pivotkI.get(), pivotkD.get());
    }
    if (armkS.hasChanged(hashCode()) || armkG.hasChanged(hashCode()) || armkV.hasChanged(hashCode())) {
      armFeedforward = new ArmFeedforward(armkS.get(), armkG.get(), armkV.get());
    }

    //!Incomplete
    if (DriverStation.isDisabled()) {
      stop();
    }

    //!Incomplete
    if (!armStopped) {
      
    }
  }

  //!Incomplete
  public void stop () {
    armStopped = true;
    intakeStopped = true;

    //setPivotVoltage(0.0);
  }

  public void setDesiredArmAngle (double angle) {
    armStopped = false;
    desiredAngle = angle;
    angle = MathUtil.clamp(angle, 0, 0); //!Update with angle constraints
    armPID.setGoal(new TrapezoidProfile.State(angle, 0.0));
  }

  public void setIntakeVoltage (double volts) {
    
  }

  public double getCurrentArmAngle () {
    return currentArmAngle;
  }

  //!Incomplete
  public void initSendable (SendableBuilder builder) {
    /// if (Constants.robotMode != RobotMode.DEVELOPMENT) return;

    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("ArmAngle", this::getCurrentArmAngle, null);
    builder.addDoubleProperty("AppliedVoltage", () -> {
      return armAppliedVoltage;
    }, null);
    builder.addDoubleProperty("DesiredAngle", () -> {
      return desiredAngle;
    }, null);
    builder.addDoubleProperty("SetpointError", () -> {
      return armPID.getPositionError();
    }, null);
  }
}
