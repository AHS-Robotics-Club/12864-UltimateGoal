package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;


public class WobbleSystem extends SubsystemBase {
    SimpleServo crServo;
    Motor motor;
    Telemetry tele;
    public WobbleSystem(SimpleServo pickMeUpDaddy, Motor mator, Telemetry telemetry){
        crServo = pickMeUpDaddy;
        motor = mator;
        tele = telemetry;
    }

    public void spinMeRightRoundBaby(){
        crServo.turnToAngle(18);
    }
    public void putMeDownUwU(){
        crServo.turnToAngle(90);
    }
    public void motorUp(){
        motor.resetEncoder();
        motor.setPositionCoefficient(0.008);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(370);
        while(!motor.atTargetPosition())
            motor.set(0.8);
        motorStop();
    }
    public void motorDown(){
        motor.resetEncoder();
        motor.setPositionCoefficient(-0.01);
        motor.setPositionTolerance(10);
        motor.setTargetPosition(-360);
        while(!motor.atTargetPosition())
            motor.set(-0.3);
        motorStop();
    }
    public void motorStop(){
        motor.resetEncoder();
        motor.stopMotor();
    }
    @Override
    public void periodic(){
        tele.addData("Motor Position", motor.getCurrentPosition());
        tele.update();

    }
}

