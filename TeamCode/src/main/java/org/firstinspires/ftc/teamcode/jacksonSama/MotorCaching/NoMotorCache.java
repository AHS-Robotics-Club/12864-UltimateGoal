package org.firstinspires.ftc.teamcode.jacksonSama.MotorCaching;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name="No Motor Cache Test")
public class NoMotorCache extends LinearOpMode {

    private DcMotor motor;
    private static final double SPEED_TOL = 0.05;
    private double setPower, start, currentStamp;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "test");
        long numLoops = 0;
        waitForStart();
        start = (double)System.nanoTime() * 1E-9;
        while (opModeIsActive() && setPower <= 1) {
            motor.setPower(setPower);
            setPower += 0.01;
            currentStamp = (double)System.nanoTime() * 1E-9;
            telemetry.addData("Loop Time", (currentStamp - start)/++numLoops);
            telemetry.update();
        }
    }

}