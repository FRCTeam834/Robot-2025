// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private Spark blinkin;
  private final int LED_PWM_PORT = 0;

  private Color defaultColor = Color.BLUE;

  public static enum Color {
    BLUE(-0.41),
    GREEN(0.77),
    CONFETTI(-0.87),
    RED(0.61),
    STROBEBLUE(-0.09),
    STROBEWHITE(-0.05),
    WHITE(0.93),
    STROBERED(-0.11);

    public final double signal;

    private Color(double signal) {
      this.signal = signal;
    }
  }

  public LEDs() {
    blinkin = new Spark(LED_PWM_PORT);
    setLEDColor(Color.WHITE);
  }

  public void setLEDColor(Color color) {
    blinkin.set(color.signal);
  }


}
