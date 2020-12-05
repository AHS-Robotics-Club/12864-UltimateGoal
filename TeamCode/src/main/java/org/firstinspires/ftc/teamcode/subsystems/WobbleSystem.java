package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleopComp2;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;


public class WobbleSystem extends SubsystemBase {
    CRServo crServo;
    Motor motor;
    Telemetry tele;
    BooleanSupplier stopRequested;
    public WobbleSystem(CRServo pickMeUpDaddy, Motor mator, Telemetry telemetry, BooleanSupplier isStopRequested){
        crServo = pickMeUpDaddy;
        motor = mator;
        tele = telemetry;
        stopRequested = isStopRequested;
    }


    public void spinMeRightRoundBaby(){
//        crServo.turnToAngle(18);
        crServo.set(1.0);
    }
    public void putMeDownUwU(){
//        crServo.turnToAngle(90);
        crServo.set(-0.2);
    }
    public void motorUp(){
        motor.resetEncoder();
        motor.setPositionCoefficient(0.008);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(372);
        while(!motor.atTargetPosition()) {
            if (stopRequested.getAsBoolean()) {
                motor.stopMotor();
                break;
            }
            motor.set(0.45);
        }
        motor.stopMotor();
    }
    public void motorDown(){
        motor.resetEncoder();
        motor.setPositionCoefficient(-0.01);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(-340);
        while(!motor.atTargetPosition()) {
            if (stopRequested.getAsBoolean()) {
                motor.stopMotor();
                break;
            }
            motor.set(-0.15);
        }
        motor.stopMotor();
    }
    public void motorStop(){
        motor.resetEncoder();
        motor.stopMotor();
    }
    public void servoStop(){
        crServo.stop();
    }
    @Override
    public void periodic(){
        tele.addData("Motor Position", motor.getCurrentPosition());
        tele.update();

    }
}

