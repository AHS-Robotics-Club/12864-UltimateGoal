package org.firstinspires.ftc.teamcode.testingFolder.forJackson;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="ShooterExample")
public class ShooterOp extends CommandOpMode {

    //Motors and Servos
    private Motor shooter;
    private SimpleServo servo;
    //Subsystems
    private ShooterSubsystem shooterSubsystem;
    //Commands
    private FlyWheelCommand flyWheelCommand;
    //Gamepad
    private GamepadEx m_driverOp;

    @Override
    public void initialize() {
        //Motors and Servos
        shooter = new Motor(hardwareMap, "shooter");
        servo = new SimpleServo(hardwareMap, "pusher", -20, 180, AngleUnit.DEGREES);
        //Gamepad
        m_driverOp = new GamepadEx(gamepad1);
        //ShooterSubsystem and Command
        shooterSubsystem = new ShooterSubsystem(shooter, servo);
        flyWheelCommand = new FlyWheelCommand(shooterSubsystem);

        //Starts and stops shooter when the A button is pressed
        m_driverOp.getGamepadButton(GamepadKeys.Button.A).toggleWhenPressed(flyWheelCommand);

    }
}
