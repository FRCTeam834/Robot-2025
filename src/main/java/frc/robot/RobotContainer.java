// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DriveWithSpeeds;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.Gyro;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class RobotContainer {

  public static DriveTrain driveTrain = new DriveTrain(
<<<<<<< HEAD
    new SwerveModule(0, 0, 0, SwerveConstants.CAN_CODER_OFFSET_FL), 
    new SwerveModule(0, 0, 0, SwerveConstants.CAN_CODER_OFFSET_FR), 
    new SwerveModule(0, 0, 0, SwerveConstants.CAN_CODER_OFFSET_BL), 
    new SwerveModule(0, 0, 0, SwerveConstants.CAN_CODER_OFFSET_BR),
=======
    new SwerveModule(3, 2, 10, SwerveConstants.CAN_CODER_OFFSET_FL), 
    new SwerveModule(5, 4, 11, SwerveConstants.CAN_CODER_OFFSET_FR), 
    new SwerveModule(7, 6, 12, SwerveConstants.CAN_CODER_OFFSET_BL), 
    new SwerveModule(9, 8, 13, SwerveConstants.CAN_CODER_OFFSET_BR),
>>>>>>> 69ab3b2 (test)
    new Gyro()
  );

  public RobotContainer() {
    driveTrain.setDefaultCommand(new DriveWithSpeeds(
      driveTrain,
      OI::getRightJoystickY,
      OI::getRightJoystickX,
      OI::getLeftJoystickX
    ));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
