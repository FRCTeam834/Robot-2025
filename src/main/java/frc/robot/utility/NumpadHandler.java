// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.OI;

public class NumpadHandler extends SubsystemBase {
  JoystickButton one = new JoystickButton(OI.numpad, 1);
  JoystickButton two = new JoystickButton(OI.numpad, 2);
  JoystickButton three = new JoystickButton(OI.numpad, 3);
  JoystickButton four = new JoystickButton(OI.numpad, 4);

  private int toggledLevel = 0;

  public NumpadHandler() {
    one.onTrue(new InstantCommand(() -> { setToggledLevel(1); }));
    two.onTrue(new InstantCommand(() -> { setToggledLevel(2); }));
    three.onTrue(new InstantCommand(() -> { setToggledLevel(3); }));
    four.onTrue(new InstantCommand(() -> { setToggledLevel(4); }));
  }

  public void setToggledLevel(int level) {
    toggledLevel = level;
  }

  public int getLevel() {
    return toggledLevel;
  }
}
