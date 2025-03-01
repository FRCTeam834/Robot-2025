package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utility.UnitQuad;

/** Stole from 2024 */
public class OI {
    public static final Joystick leftJoystick = new Joystick(0);
    public static final Joystick rightJoystick = new Joystick(1);
    public static final XboxController xbox = new XboxController(2);

    // keypad
    public static final XboxController keypad0To7 = new XboxController(3);
    public static final XboxController keypad8ToMinus = new XboxController(4);

    public static final double flightJoystickDeadzone = 0.05;
    public static final double xboxJoystickDeadzone = 0.1;

    /**
     * @return left joystick x input
     */
    public static final double getLeftJoystickX () {
        double raw = leftJoystick.getX();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return left joystick y input
     */
    public static final double getLeftJoystickY () {
        double raw = -leftJoystick.getY();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return right joystick x input
     */
    public static final double getRightJoystickX () {
        double raw = rightJoystick.getX();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return right joystick y input
     */
    public static final double getRightJoystickY () {
        double raw = -rightJoystick.getY();
        if (Math.abs(raw) < flightJoystickDeadzone) raw = 0.0;
        return UnitQuad.calculate(raw);
    }
    
    /**
     * @return right joystick trigger state
     */
    public static final boolean isRightJoystickTriggerPressed() {
        return rightJoystick.getTrigger();
    }

    public static final boolean isDPadUpPressed() {
        return xbox.getPOV() == 0;
    }

    public static final boolean isDPadDownPressed() {
        return xbox.getPOV() == 180;
    }

    public static final double getXboxLeftJoystickY () {
        double raw = xbox.getLeftY();
        if (Math.abs(raw) < xboxJoystickDeadzone) raw = 0.0;
        return raw;
    }

    public static final double getXboxRightJoystickY () {
        double raw = xbox.getRightY();
        if (Math.abs(raw) < xboxJoystickDeadzone) raw = 0.0;
        return raw;
    }

    /**
     * 
     * keypad bindings (set in keyboard splitter)
     */
    public static final JoystickButton getKeypad0 () {
        return new JoystickButton(keypad0To7, 8);
    }

    public static final JoystickButton getKeypad1 () {
        return new JoystickButton(keypad0To7, 7);
    }

    public static final JoystickButton getKeypad2 () {
        return new JoystickButton(keypad0To7, 5);
    }

    public static final JoystickButton getKeypad3 () {
        return new JoystickButton(keypad0To7, 6);
    }

    public static final JoystickButton getKeypad4 () {
        return new JoystickButton(keypad0To7, 1);
    }

    public static final JoystickButton getKeypad5 () {
        return new JoystickButton(keypad0To7, 2);
    }

    public static final JoystickButton getKeypad6 () {
        return new JoystickButton(keypad0To7, 3);
    }

    public static final JoystickButton getKeypad7 () {
        return new JoystickButton(keypad0To7, 4);
    }

    public static final JoystickButton getKeypad8 () {
        return new JoystickButton(keypad8ToMinus, 8);
    }

    public static final JoystickButton getKeypad9 () {
        return new JoystickButton(keypad8ToMinus, 7);
    }

    public static final JoystickButton getKeypadDel () {
        return new JoystickButton(keypad8ToMinus, 1);
    }

    public static final JoystickButton getKeypadEnter () {
        return new JoystickButton(keypad8ToMinus, 6);
    }

    public static final JoystickButton getKeypadPlus () {
        return new JoystickButton(keypad8ToMinus, 5);
    }

    public static final JoystickButton getKeypadNum () {
        return new JoystickButton(keypad8ToMinus, 2);
    }

    public static final JoystickButton getKeypadSlash () {
        return new JoystickButton(keypad8ToMinus, 3);
    }

    public static final JoystickButton getKeypadAsterisk () {
        return new JoystickButton(keypad8ToMinus, 4);
    }

    /**
     * Not enough emulated buttons, so this is a boolean
     * @return
     */
    public static final Boolean getKeypadMinus () {
        return keypad8ToMinus.getPOV() == 0;
    }
}