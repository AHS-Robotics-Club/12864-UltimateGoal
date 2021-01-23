package org.firstinspires.ftc.teamcode.testingFolder.shhhnopeaking.combomod;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class ComboGamepad extends GamepadEx {

    public Gamepad gamepad;

    private HashMap<GamepadKeys.Button, ButtonReader> buttonReaders;
    private HashMap<GamepadKeys.Button, GamepadButton> gamepadButtons;

    private final GamepadKeys.Button[] buttons = {
            GamepadKeys.Button.Y, GamepadKeys.Button.X, GamepadKeys.Button.A, GamepadKeys.Button.B, GamepadKeys.Button.LEFT_BUMPER, GamepadKeys.Button.RIGHT_BUMPER, GamepadKeys.Button.BACK,
            GamepadKeys.Button.START, GamepadKeys.Button.DPAD_UP, GamepadKeys.Button.DPAD_DOWN, GamepadKeys.Button.DPAD_LEFT, GamepadKeys.Button.DPAD_RIGHT,
            GamepadKeys.Button.LEFT_STICK_BUTTON, GamepadKeys.Button.RIGHT_STICK_BUTTON
    };

    /**
     * The constructor, that contains the gamepad object from the
     * opmode.
     *
     * @param gamepad the gamepad object from the opmode
     */
    public ComboGamepad(Gamepad gamepad) {
        super(gamepad);
    }


}
