package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;


public class WobbleSystem extends SubsystemBase {
    CRServo crServo;
    Motor motor;
    Telemetry tele;
    public WobbleSystem(CRServo pickMeUpDaddy, Motor mator, Telemetry telemetry){
        crServo = pickMeUpDaddy;
        motor = mator;
        tele = telemetry;

        motor.resetEncoder();
    }
    public void spinMeRightRoundBaby(){
//        crServo.turnToAngle(18);
        crServo.set(1.0);
    }
    public void putMeDownUwU(){
//        crServo.turnToAngle(90);
        crServo.set(-0.2);
    }
    public void motorStop(){
        motor.resetEncoder();
        motor.stopMotor();
    }
    public void servoStop(){
        crServo.stop();
    }
    public Motor getMotor(){
        return motor;
    }
    @Override
    public void periodic(){
        tele.addData("Motor Position", motor.getCurrentPosition());
        tele.update();

    }
}

