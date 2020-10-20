package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Odom>10-20")
public class TeleOpOdomInOuttakeKunal extends OpMode {

    private Motor fL, bL, fR, bR, shooter, intake;
    private MecanumDrive drive;
    private RevIMU imu;
    GamepadEx cont;
    DifferentialOdometry odom;
    MotorEx encodLeft, encodRight;
    static final double TRACKWIDTH = 1; //Change -- //Distance between both Encoders
    static final double TICKS_TO_INCHES = 1; //Change
    @Override
    public void init() {
        //motors
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        shooter = new Motor(hardwareMap, "shooter");
        intake = new Motor(hardwareMap, "intake");
        /*
        //encoders
        eL = new Motor(hardwareMap, "eL");
        eR = new Motor(hardwareMap, "eR");
         */
        encodLeft = new MotorEx(hardwareMap, "encod_L");
        encodRight = new MotorEx(hardwareMap, "encod_R");

        //Is this even neccessary?????
        fL.set(0);
        fR.set(0);
        bL.set(0);
        bR.set(0);
        shooter.set(0);
        intake.set(0);


        drive = new MecanumDrive(fL, fR, bL, bR);
        imu = new RevIMU(hardwareMap);

        //Useful
        //odom = new DifferentialOdometry();
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //imu.init(parameters);
        cont = new GamepadEx(gamepad1);


        //odemetry set up
        odom = new DifferentialOdometry(
            () -> encodLeft.getCurrentPosition() * TICKS_TO_INCHES,
            () -> encodRight.getCurrentPosition() * TICKS_TO_INCHES,
            TRACKWIDTH
        );
    }
    @Override
    public void loop() {


        /*

        ANOTHER WAY OF driveRobotCentric

        double speedForward = gamepad1.left_trigger < 0.5 ? 2 * gamepad1.left_trigger : 2* (-gamepad1.left_trigger * 0.5)
        drive.driveRobotCentric(cont.getLeftX(), speedForward, cont.getRightX());

        */
        odom.updatePose();



        drive.driveRobotCentric(cont.getLeftX(), cont.getLeftY(), cont.getRightX());

        if (gamepad1.dpad_up){
            shooter.set(1.0);
        } else if (gamepad1.dpad_down) {
            shooter.set(0.8);
        } else if (gamepad1.dpad_right) {
            shooter.set(0.6);
        } else if (gamepad1.dpad_left){
            shooter.set(0.4);
        } else {
            shooter.set(0);
        }
        if(gamepad1.right_trigger > 0.1){
            intake.set(1.0);
        }
        telemetry.addData("StrafeSpeed", cont.getLeftX());
        telemetry.addData("ForwardSpeed", cont.getLeftY());
        telemetry.addData("TurnSpeed", cont.getRightX());
        telemetry.update();
    }
}
