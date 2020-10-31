package org.firstinspires.ftc.teamcode.jacksonSama;

import android.media.MediaPlayer;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="Jackson Homosexual")
public class TestingForJackson extends LinearOpMode {

    Motor motor;
    PIDController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("hello", "hello");
        telemetry.update();
        motor = new Motor(hardwareMap, "test", Motor.GoBILDA.RPM_312);
//        controller = new PIDController(0.05, 0.4, 0.0001);
            controller = new PIDController(1, 1, 0);
        controller.setSetPoint(10000);
        controller.calculate();
        motor.resetEncoder();

        controller.setTolerance(5);
        waitForStart();
        while (opModeIsActive() && !controller.atSetPoint()) {
            motor.set(controller.calculate(motor.getCurrentPosition()));
            telemetry.addData("Motor position", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.stopMotor();


    }

}