package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.Inputs.*;
import static frc.robot.Robot.drivetrain;

public class OI {

    public static Joystick leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);

    public static final JoystickButton shiftUp = new JoystickButton(leftJoystick, SHIFT_UP_PORT);
    public static final JoystickButton shiftDown = new JoystickButton(leftJoystick, SHIFT_DOWN_PORT);

    static {
        shiftUp.whenPressed(() -> drivetrain.shiftUp());
        shiftDown.whenPressed(() -> drivetrain.shiftDown());
    }

}
