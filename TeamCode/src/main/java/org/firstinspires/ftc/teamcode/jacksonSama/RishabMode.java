package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="bahsir")
public class RishabMode extends LinearOpMode {
    Motor motor;
    GamepadEx m_driverOp;
    ToggleButtonReader poopoo;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = new Motor(hardwareMap, "Youre bad at code");
        m_driverOp = new GamepadEx(gamepad1);
        poopoo = new ToggleButtonReader(m_driverOp, GamepadKeys.Button.A);
        waitForStart();
        while(opModeIsActive()){
            if(poopoo.getState() == true){
                motor.set(1);
            }else{
                motor.set(0);
            }
            poopoo.readValue();
        }
    }
}
