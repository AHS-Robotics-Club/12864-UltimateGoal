package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Ethan gay")
public class TurnServo extends LinearOpMode {
    CRServo servo;
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "push");
        motor = hardwareMap.get(DcMotor.class, "shot");
        waitForStart();
        motor.setPower(1);
        servo.setPower(0.2);
        sleep(100);
        servo.setPower(0);
        sleep(200);
        servo.setPower(-0.05);
        sleep(100);
        servo.setPower(0);
        motor.setPower(1);
        sleep(1000);
    }
}
