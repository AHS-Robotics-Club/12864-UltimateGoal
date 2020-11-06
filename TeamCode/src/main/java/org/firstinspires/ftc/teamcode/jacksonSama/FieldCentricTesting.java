package org.firstinspires.ftc.teamcode.jacksonSama;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Field Centric Testing")
public class FieldCentricTesting extends LinearOpMode {

    private RevIMU imu;
    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new RevIMU(hardwareMap);
        imu.init();
        driverOp = new GamepadEx(gamepad1);
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "fR");
        drive = new MecanumDrive(fL, fR, bL, bR);
        waitForStart();
        while (opModeIsActive()) {
            drive.driveFieldCentric(-driverOp.getLeftX(), driverOp.getLeftY(), -driverOp.getRightX(), (imu.getAbsoluteHeading() + 30));
            telemetry.addData("Heading", imu.getAbsoluteHeading() + 30);
            telemetry.update();
        }
    }

}