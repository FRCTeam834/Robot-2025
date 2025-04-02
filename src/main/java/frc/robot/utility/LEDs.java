// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private Spark blinkin;
  private final int LED_PWM_PORT = 0;

  public ledColor defaultColor = ledColor.WHITE;
  private ledColor timedColor = null;
  private final Timer timer = new Timer();
  private double timedSeconds = 0.0;

  public static enum ledColor {
    BLUE(-0.41),
    GREEN(0.77),
    CONFETTI(-0.87),
    RED(0.61),
    STROBEBLUE(-0.09),
    STROBEWHITE(-0.05),
    STROBEGOLD(-0.07),
    WHITE(0.93),
    FIRE(-0.57),
    STROBERED(-0.11);

    public final double signal;

    private ledColor(double signal) {
      this.signal = signal;
    }
  }

  public LEDs() {
    blinkin = new Spark(LED_PWM_PORT);
    setLEDColor(defaultColor);
  }

  public void setColorForTime(ledColor color, double seconds) {
    timer.reset();
    timer.start();
    timedSeconds = seconds;
    timedColor = color;
    setLEDColor(color);
  }

  public void cancelColorForTime () {
    timedSeconds = 0.0;
  }

  public void setLEDColor(ledColor color) {
    blinkin.set(color.signal);
  }

  @Override
  public void periodic() {
    if (timedColor != null && timer.hasElapsed(timedSeconds)) {
      timedColor = null;
      timer.reset();
      timer.stop();
      setLEDColor(defaultColor);
    }
  }
}
