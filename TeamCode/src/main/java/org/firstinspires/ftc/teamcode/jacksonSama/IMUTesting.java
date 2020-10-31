package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="IMU Testing")
public class IMUTesting extends LinearOpMode {

    private RevIMU imu;
//for some reason we have to init RevIMU in runOpMode for some reason weirdchamp
    @Override
    public void runOpMode() throws InterruptedException {
        imu = new RevIMU(hardwareMap);
        imu.init();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Absolute Heading", imu.getAbsoluteHeading());
            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Axis 1", imu.getAngles()[0]);
            telemetry.addData("Axis 2", imu.getAngles()[1]);
            telemetry.addData("Axis 3", imu.getAngles()[2]);
            telemetry.update();
        }
    }

}