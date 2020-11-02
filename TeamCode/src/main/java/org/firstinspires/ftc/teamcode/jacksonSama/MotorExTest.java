package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MotorEx testing")
public class MotorExTest extends LinearOpMode {

    private MotorEx test;

    @Override
    public void runOpMode() throws InterruptedException {
        test = new MotorEx(hardwareMap, "test", Motor.GoBILDA.RPM_312);
        test.setRunMode(Motor.RunMode.VelocityControl);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                test.setVelocity(312.0 / 60 * 537.6 * 0.9);
            }
            else test.setVelocity(0);
        }
        test.setVelocity(0);
    }

}