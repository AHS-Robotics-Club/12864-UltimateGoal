package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MotorEx testing")
public class MotorExTest extends LinearOpMode {

    private MotorEx test;

    @Override
    public void runOpMode() throws InterruptedException {
        test = new MotorEx(hardwareMap, "shot");
        test.setRunMode(Motor.RunMode.VelocityControl);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                test.set(1);
            }
            else test.stopMotor();
        }
        test.stopMotor();
    }

}

