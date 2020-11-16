package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

@Autonomous(name="AutonMain")
public class AutonPlsWorkOwO extends LinearOpMode {

    private MotorEx fL, fR, bL, bR;
    private MotorEx encoderLeft, encoderRight;
    private MotorGroup left, right;
    private DifferentialDrive diffy;
    private DifferentialOdometry diffyOdom;
    private PIDController xCont, yCont, hCont;
    private static final double TRACKWIDTH = 13.4;
    private static double TICKS_TO_INCHES;
    private static final double WHEEL_DIAMETER = 4.0;
    private static double TICKS_PER_REV;
    private Timing.Timer timeywimey = new Timing.Timer(5,TimeUnit.SECONDS);


    @Override
    public void runOpMode() throws InterruptedException {
        fL = new MotorEx(hardwareMap, "fL", Motor.GoBILDA.RPM_435);
        fR = new MotorEx(hardwareMap, "fR", Motor.GoBILDA.RPM_435);
        bL = new MotorEx(hardwareMap, "bL", Motor.GoBILDA.RPM_435);
        bR = new MotorEx(hardwareMap, "bR", Motor.GoBILDA.RPM_435);

        bL.setInverted(true);
        TICKS_PER_REV = fL.getCPR();
        TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

        left = new MotorGroup(fL, bL);
        right = new MotorGroup(fR, bR);

        Motor.Encoder leftEncoder = fL.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        Motor.Encoder rightEncoder = fR.encoder.setDistancePerPulse(TICKS_TO_INCHES);
        rightEncoder.setDirection(Motor.Direction.REVERSE);

        leftEncoder.reset();
        rightEncoder.reset();
        //TODO: Tune when robot can drive
        xCont = new PIDController(0.06, 0.4, 0.0001);
        yCont = new PIDController(0.06, 0.3, 0.0001);
        hCont = new PIDController(0.06, 0.3, 0);


        diffy = new DifferentialDrive(left, right);
        diffyOdom = new DifferentialOdometry(leftEncoder::getDistance, rightEncoder::getDistance, TRACKWIDTH);
        waitForStart();
        timeywimey.start();
        xCont.setSetPoint(2);
        telemetry.addData("Right Motor position", rightEncoder.getPosition());
        telemetry.addData("Left Motor position", leftEncoder.getPosition());
        telemetry.update();
        hCont.setSetPoint(Math.PI);
        do {
            telemetry.addData("Right Motor position", rightEncoder.getPosition());
            telemetry.addData("Left Motor position", leftEncoder.getPosition());
            telemetry.update();
            if(timeywimey.done()){
                break;
            }
            if (isStopRequested()) break;
            diffy.arcadeDrive(
                    xCont.calculate(diffyOdom.getPose().getX()),
                    hCont.calculate(diffyOdom.getPose().getHeading())
            );
            diffyOdom.updatePose();
        } while (opModeIsActive() && (!xCont.atSetPoint() || !hCont.atSetPoint()));

        yCont.setSetPoint(0);
        telemetry.addData("Right Motor position", rightEncoder.getPosition());
        telemetry.addData("Left Motor position", leftEncoder.getPosition());
        telemetry.update();
        do {
            telemetry.addData("Right Motor position", rightEncoder.getPosition());
            telemetry.addData("Left Motor position", leftEncoder.getPosition());
            telemetry.update();
            if (isStopRequested()) break;
            diffy.arcadeDrive(yCont.calculate(diffyOdom.getPose().getY()), 0);
            diffyOdom.updatePose();
        } while (opModeIsActive() && !yCont.atSetPoint());
        telemetry.update();
    }
}
