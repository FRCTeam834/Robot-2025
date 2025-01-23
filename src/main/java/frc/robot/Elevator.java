// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  //Motor
  private final CANSparkMax elevatorMotor1;
  private final CANSparkMax elevatorMotor2;

  //Elevator absolute encoder
  private final AbsoluteEncoder elevatorEncoder;

  //Boolean for if the elevator is stopped
  private boolean elevatorStopped = true;

  //PID constants
  private static TunableNumber elevatorkP = new TunableNumber("Elevator/ElevatorkP");
  private static TunableNumber elevatorkI = new TunableNumber("Elevator/ElevatorkI");
  private static TunableNumber elevatorkD = new TunableNumber("Elevator/ElevatorkD");

  //Feedforward constants
  private static TunableNumber elevatorkS = new TunableNumber("Elevator/ElevatorkS"); //static friction
  private static TunableNumber elevatorkG = new TunableNumber("Elevator/ElevatorkG"); //gravity
  private static TunableNumber elevatorkV = new TunableNumber("Elevator/ElevatorkV"); //velocity
  private static TunableNumber elevatorkA = new TunableNumber("Elevator/ElevatorkA"); //acceleration

  //Initialize PID controller
  private ProfiledPIDController elevatorPID = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0)); //!Set constraints
  private double desiredPosition = 0.0;

  //Initialize feedforward
  private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);

  static {
    //!Initialize default constants
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
    //!Update with correct motor IDs
    elevatorMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    elevatorMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    configureSpark("", () -> { return elevatorMotor1.restoreFactoryDefaults(); });
    configureSpark("", () -> { return elevatorMotor2.restoreFactoryDefaults(); });
    elevatorEncoder = elevatorMotor1.getAbsoluteEncoder(Type.kDutyCycle);
    
    //!Update with correct configurations for the elevator encoder and the elevator motor
    /*
    configureSpark("", () -> { return elevatorEncoder.setPositionConversionFactor(1.0); });
    configureSpark("", () -> { return elevatorEncoder.setVelocityConversionFactor(1.0); });
    configureSpark("", () -> { return elevatorEncoder.setZeroOffset(0); });
    configureSpark("", () -> { return elevatorMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); }
    configureSpark("", () -> { return elevatorMotor1.enableVoltageCompensation(0.0); });
    configureSpark("", () -> { return elevatorMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10); }
    configureSpark("", () -> { return elevatorMotor2.enableVoltageCompensation(0.0); });
    */
    elevatorMotor2.follow(elevatorMotor1); //Motor 2's output is the same as Motor 1
    
    elevatorPID.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putData(this);

    if(/*Robot is in competition mode*/) { //!Update
      //! Delay needed?
      Timer.delay(0.25);
      //BURN FLASH
      configureSpark("", () -> { return elevatorMotor1.burnFlash(); });
      Timer.delay(0.25);
      configureSpark("", () -> { return elevatorMotor2.burnFlash(); });
      Timer.delay(0.25);
    }

  }

  //Configure the spark maxes and report any errors
  public static boolean configureSpark(String message, Supplier<REVLibError> config) {
    REVLibError err = REVLibError.kOk;
    for (int i = 0; i < 15; i++) {
        err = config.get();
        if (err == REVLibError.kOk) {
            return true;
        }
        Timer.delay(0.05);
    }

    DriverStation.reportError(String.format(
        "[MergeError] - CANSparkMax failed to configure setting. MergeMessage: %s. Spark error code: %s \nSee stack trace below.", 
        message,
        err.toString()), 
        true);
        
    return false;
  }

  @Override
  public void setIdleCoast() {
    elevatorMotor1.setIdleMode(IdleMode.kCoast);
    elevatorMotor2.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void setIdleBrake() {
    elevatorMotor1.setIdleMode(IdleMode.kBrake);
    elevatorMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void setElevatorVoltage(double volts) {
    elevatorMotor1.setVoltage(volts);
  }


  //Called once per scheduler run
  @Override
  public void periodic() {
    
    //Update the elevator PID if the constants have changed
    if (elevatorkP.hasChanged(hashCode()) || elevatorkI.hasChanged(hashCode()) || elevatorkD.hasChanged(hashCode())) {
      elevatorPID.setPID(elevatorkP.get(), elevatorkI.get(), elevatorkD.get());
    }

    //Update the elevator feedforward if the constants have changed
    if (elevatorkS.hasChanged(hashCode()) || elevatorkG.hasChanged(hashCode()) || elevatorkV.hasChanged(hashCode()) || elevatorkA.hasChanged(hashCode())) {
      elevatorFeedforward = new elevatorFeedforward(elevatorkS.get(), elevatorkG.get(), elevatorkV.get(), elevatorkA.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }

    if (!elevatorStopped) {
      //Set the voltage of the motor
      setElevatorVoltage(
        elevatorFeedforward.calculate(elevatorPID.getSetpoint().position, elevatorPID.getSetpoint().velocity) +
        elevatorPID.calculate(getCurrentElevatorPosition()));
    }

    public void stop () {
      elevatorStopped = true;
      
      //Set voltage to 0
      setElevatorVoltage(0);
    }

    public void setDesiredElevatorPosition (double position) {
      elevatorStopped = false;
      desiredPosition = position;
      
      //Restrict the posible positions of the elevator
      position = MathUtil.clamp(position, 0, 1); //! Put values for the range of positions
      elevatorPID.setGoal(new TrapezoidProfile.State(position, 0.0));
    }
    

    public double getCurrentElevatorPosition () {
      return elevatorEncoder.getPosition();
    }
    
    @Override
    public void initSendable (SendableBuilder builder) {

      builder.setSmartDashboardType("Elevator");

      builder.addDoubleProperty("ElevatorPosition", this::getCurrentElevatorPosition, null);

      builder.addDoubleProperty("AppliedVoltage", () -> {
        return elevatorMotor1.getAppliedOutput();
      }, null);
      builder.addDoubleProperty("DesiredPosition", () -> {
        return desiredPosition;
      }, null);
      builder.addDoubleProperty("SetpointError", () -> {
        return elevatorPID.getPositionError();
      }, null);
  }
  }
}
