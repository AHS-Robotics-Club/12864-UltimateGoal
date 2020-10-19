package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kanavkool")
public class TeleOpCool extends OpMode {
    private Motor fL, bL, fR, bR, eL, eR;
    private MecanumDrive drive;
    private RevIMU imu;
    GamepadEx cont;
    DifferentialOdometry odom;

    //Kanav is no longer cool
    @Override
    public void init() {
        //motors
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        //encoders
        eL = new Motor(hardwareMap, "eL");
        eR = new Motor(hardwareMap, "eR");

        drive = new MecanumDrive(fL, fR, bL, bR);
        imu = new RevIMU(hardwareMap);
//        odom = new DifferentialOdometry();
        
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.init(parameters);
        cont = new GamepadEx(gamepad1);
    }
    @Override
    public void loop() {
        drive.driveRobotCentric(cont.getLeftX(), cont.getLeftY(), cont.getRightX());

        telemetry.addData("StrafeSpeed", cont.getLeftX());
        telemetry.addData("ForwardSpeed", cont.getLeftY());
        telemetry.addData("TurnSpeed", cont.getRightX());
        telemetry.update();
    }
}