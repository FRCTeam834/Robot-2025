// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Gyro extends SubsystemBase {
  
  private final Pigeon2 pigeon;

  public Gyro() {
    pigeon = new Pigeon2(SwerveConstants.PIGEON_ID); //TODO: replace id
    pigeon.setYaw(0.0);
  }

  public double getYaw() {
    return Math.IEEEremainder(pigeon.getYaw().getValueAsDouble(), 360);
  }

  public double getAngularVelo() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

  public void resetYaw(double angle) {
    pigeon.setYaw(angle);
  }
}
